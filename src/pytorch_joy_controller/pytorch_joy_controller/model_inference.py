import numpy as np
import torch
from .model.model import DNN  # モデル定義をインポート


class Model:
    def __init__(self, path):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(self.device)
        # モデルのインスタンスを作成
        self.model = DNN()
        
        # 重みを読み込み（weights_only=Trueがデフォルトで安全）
        self.model.load_state_dict(torch.load(path, map_location=self.device, weights_only=True))
        
        # デバイスに転送して評価モードに設定
        self.model.to(self.device)
        self.model.eval()

    def inference(self, scan_ranges: np.ndarray) -> list:
        input_data = torch.from_numpy(np.array(scan_ranges, dtype=np.float32)).reshape(1, -1)
        input_data = input_data.to(self.device)

        with torch.no_grad():
            output = self.model(input_data)

        joy_axes = output.cpu().numpy().flatten()
        return joy_axes.tolist()


if __name__ == "__main__":
    dummy_input = np.random.rand(1, 1081).astype(np.float32)
    model = Model("./model/model.pth")
    output = model.inference(dummy_input)
    print(output)
