import numpy as np
import torch
import torch.nn as nn


class DNN(nn.Module):
    def __init__(self, input_dim=1081, output_dim=2, history_length=4, prediction_steps=1):
        """
        Args:
            input_dim: 入力スキャンデータの次元数
            output_dim: 各ステップの出力次元（speed, angleで2）
            history_length: 入力履歴の長さ
            prediction_steps: 予測する未来のステップ数
        """
        super().__init__()
        self.history_length = history_length
        self.prediction_steps = prediction_steps

        # 入力チャネル数を履歴長に変更
        self.conv1 = nn.Conv1d(history_length, 24, kernel_size=5, stride=2, padding=2)
        self.conv2 = nn.Conv1d(24, 36, kernel_size=5, stride=2, padding=2)
        self.conv3 = nn.Conv1d(36, 48, kernel_size=5, stride=2, padding=2)
        self.conv4 = nn.Conv1d(48, 64, kernel_size=3, stride=1, padding=1)
        self.conv5 = nn.Conv1d(64, 64, kernel_size=3, stride=1, padding=1)

        def conv1d_out_len(L, kernel, pad, stride):
            return (L + 2 * pad - kernel) // stride + 1

        L = input_dim
        for k, p, s in [(5, 2, 2), (5, 2, 2), (5, 2, 2), (3, 1, 1), (3, 1, 1)]:
            L = conv1d_out_len(L, k, p, s)
        flatten_dim = L * 64

        self.fc1 = nn.Linear(flatten_dim, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        # 出力を prediction_steps * output_dim に変更
        self.fc4 = nn.Linear(10, prediction_steps * output_dim)

        self.relu = nn.ReLU()

    def forward(self, x):
        # x の形状: (batch_size, history_length, scan_dim)
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.relu(self.conv4(x))
        x = self.relu(self.conv5(x))
        x = x.flatten(1)
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.relu(self.fc3(x))
        x = self.fc4(x)
        
        # (batch_size, prediction_steps * output_dim) -> (batch_size, prediction_steps, output_dim)
        batch_size = x.size(0)
        x = x.view(batch_size, self.prediction_steps, -1)
        
        return x


class Model:
    def __init__(self, path, prediction_steps=1):
        """
        Args:
            path: モデルの重みファイルのパス
            prediction_steps: 予測ステップ数（学習時と同じ値を指定）
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.prediction_steps = prediction_steps

        # モデルのインスタンスを作成
        self.model = DNN(prediction_steps=prediction_steps)

        # 重みを読み込み
        self.model.load_state_dict(
            torch.load(path, map_location=self.device, weights_only=True)
        )

        # デバイスに転送して評価モードに設定
        self.model.to(self.device)
        self.model.eval()

    def inference(self, scan_ranges: np.ndarray) -> list:
        """
        推論を実行
        
        Args:
            scan_ranges: スキャンデータ (history_length, scan_dim) の形状
        
        Returns:
            予測結果のリスト
            - prediction_steps=1: [speed, angle]
            - prediction_steps>1: [[speed1, angle1], [speed2, angle2], ...]
        """
        input_data = torch.from_numpy(
            np.array(scan_ranges, dtype=np.float32)
        ).unsqueeze(0)
        input_data = input_data.to(self.device)

        with torch.no_grad():
            output = self.model(input_data)

        # (1, prediction_steps, 2) -> (prediction_steps, 2)
        predictions = output.cpu().numpy().squeeze(0)
        
        if self.prediction_steps == 1:
            # 単一ステップの場合は1次元リストを返す
            return predictions.flatten().tolist()
        else:
            # 複数ステップの場合は2次元リストを返す
            return predictions.tolist()

    def inference_next_step_only(self, scan_ranges: np.ndarray) -> list:
        """
        次のステップのみを予測（複数ステップモデルでも最初のステップだけ返す）
        
        Args:
            scan_ranges: スキャンデータ (history_length, scan_dim) の形状
        
        Returns:
            次ステップの予測 [speed, angle]
        """
        predictions = self.inference(scan_ranges)
        
        if self.prediction_steps == 1:
            return predictions
        else:
            return predictions[0]


if __name__ == "__main__":
    # 使用例1: 単一ステップ予測（元のmodel.pthを使用）
    print("=== Single-step prediction ===")
    dummy_input = np.random.rand(4, 1081).astype(np.float32)
    
    # 元のモデルファイルの場合（prediction_steps=1がデフォルト）
    model_single = Model("./model/model.pth", prediction_steps=1)
    output = model_single.inference(dummy_input)
    print(f"Output shape: {np.array(output).shape}")
    print(f"Output: {output}")
    