import numpy as np
import torch
import torch.nn as nn


class DNN(nn.Module):
    def __init__(
        self,
        input_dim=1081,
        output_dim=2,
        scan_history_length=4,
        prediction_steps=1,
    ):
        """
        Args:
            input_dim: 入力スキャンデータの次元数
            output_dim: 各ステップの出力次元（throttle, angleで2）
            scan_history_length: スキャンデータの履歴長
            prediction_steps: 予測する未来のステップ数
        """
        super().__init__()
        self.scan_history_length = scan_history_length
        self.prediction_steps = prediction_steps

        # スキャン履歴用の畳み込み層
        self.conv1 = nn.Conv1d(scan_history_length, 24, kernel_size=5, stride=2, padding=2)
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
        self.fc4 = nn.Linear(10, prediction_steps * output_dim)

        self.relu = nn.ReLU()

    def forward(self, x):
        """
        Args:
            x: スキャンデータ (batch_size, scan_history_length, scan_dim)
        """
        # CNNでスキャンデータを処理
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
    def __init__(
        self,
        path,
        prediction_steps=1,
        scan_history_length=4,
    ):
        """
        Args:
            path: モデルの重みファイルのパス（.pthファイル）
            prediction_steps: 予測ステップ数（学習時と同じ値を指定）
            scan_history_length: スキャン履歴長（学習時と同じ値を指定）
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.prediction_steps = prediction_steps
        self.scan_history_length = scan_history_length

        # モデルのインスタンスを作成
        self.model = DNN(
            scan_history_length=scan_history_length,
            prediction_steps=prediction_steps,
        )

        # チェックポイントを読み込み
        checkpoint = torch.load(path, map_location=self.device, weights_only=True)
        
        # モデルの重みを読み込み
        self.model.load_state_dict(checkpoint['model_state_dict'])

        # 正規化統計量を読み込み
        self.scan_mean = checkpoint.get('scan_mean', None)
        self.scan_std = checkpoint.get('scan_std', None)
        self.target_mean = checkpoint.get('target_mean', None)
        self.target_std = checkpoint.get('target_std', None)

        # デバイスに転送
        if self.scan_mean is not None:
            self.scan_mean = self.scan_mean.to(self.device)
            self.scan_std = self.scan_std.to(self.device)
        if self.target_mean is not None:
            self.target_mean = self.target_mean.to(self.device)
            self.target_std = self.target_std.to(self.device)

        # デバイスに転送して評価モードに設定
        self.model.to(self.device)
        self.model.eval()

        print(f"Model loaded from {path}")
        print(f"Normalization stats loaded: scan={self.scan_mean is not None}, "
              f"target={self.target_mean is not None}")

    def _normalize_scan(self, scan_data):
        """スキャンデータを正規化"""
        if self.scan_mean is not None and self.scan_std is not None:
            return (scan_data - self.scan_mean) / (self.scan_std + 1e-8)
        return scan_data

    def _denormalize_target(self, target_data):
        """ターゲットデータを逆正規化"""
        if self.target_mean is not None and self.target_std is not None:
            return target_data * self.target_std + self.target_mean
        return target_data

    def inference(self, scan_ranges: np.ndarray) -> list:
        """
        推論を実行

        Args:
            scan_ranges: スキャンデータ (scan_history_length, scan_dim) の形状

        Returns:
            予測結果のリスト
            - prediction_steps=1: [throttle, angle]
            - prediction_steps>1: [[throttle1, angle1], [throttle2, angle2], ...]
        """
        # 生データをテンソルに変換
        input_data = torch.from_numpy(np.array(scan_ranges, dtype=np.float32)).unsqueeze(0)
        input_data = input_data.to(self.device)

        # 正規化
        input_data = self._normalize_scan(input_data)

        # 推論
        with torch.no_grad():
            output = self.model(input_data)
            # 逆正規化して生データに戻す
            output = self._denormalize_target(output)

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
            scan_ranges: スキャンデータ (scan_history_length, scan_dim) の形状

        Returns:
            次ステップの予測 [throttle, angle]
        """
        predictions = self.inference(scan_ranges)

        if self.prediction_steps == 1:
            return predictions
        else:
            return predictions[0]

# if __name__ == "__main__":
#     # モデルの動作確認
#     model_path = "../model/model.pth"
#     model = Model(model_path, prediction_steps=40, scan_history_length=4)

#     # ダミースキャンデータ
#     dummy_scan = np.random.rand(4, 1081).astype(np.float32)*10

#     # 推論実行
#     preds = model.inference(dummy_scan)
#     print("Predictions:", preds)