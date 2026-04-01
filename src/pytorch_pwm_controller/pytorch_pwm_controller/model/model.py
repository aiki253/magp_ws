import numpy as np
import torch
import torch.nn as nn


class DNN(nn.Module):
    def __init__(
        self,
        input_dim=1081,
        output_dim=2,
        scan_history_length=1,  # 1フレームのみ使用
        prediction_steps=1,
        dropout=0.0,
    ):
        """
        TinyLidarNet ベースの1D CNN モデル（1フレーム入力）
        Args:
            input_dim: 入力スキャンデータの次元数（デフォルト: 1081）
            output_dim: 出力次元（throttle, angle で 2）
            scan_history_length: 履歴フレーム数（このモデルでは1を想定）
            prediction_steps: 予測ステップ数
            dropout: ドロップアウト率
        """
        super().__init__()
        self.scan_history_length = scan_history_length
        self.prediction_steps = prediction_steps
        self.input_dim = input_dim

        # 論文 Fig.3 に基づく 5層 1D Conv
        # input: (batch, 1, input_dim)
        self.conv_layers = nn.Sequential(
            # Conv1: kernel=10, stride=4 -> (batch, 24, (input_dim-10)//4+1)
            nn.Conv1d(in_channels=1,  out_channels=24, kernel_size=10, stride=4),
            nn.ReLU(),
            # Conv2: kernel=8, stride=4
            nn.Conv1d(in_channels=24, out_channels=36, kernel_size=8,  stride=4),
            nn.ReLU(),
            # Conv3: kernel=4, stride=2
            nn.Conv1d(in_channels=36, out_channels=48, kernel_size=4,  stride=2),
            nn.ReLU(),
            # Conv4: kernel=3, stride=1
            nn.Conv1d(in_channels=48, out_channels=64, kernel_size=3,  stride=1),
            nn.ReLU(),
            # Conv5: kernel=3, stride=1
            nn.Conv1d(in_channels=64, out_channels=64, kernel_size=3,  stride=1),
            nn.ReLU(),
        )

        # Conv後のフラット次元を自動計算
        with torch.no_grad():
            dummy = torch.zeros(1, 1, input_dim)
            conv_out_dim = self.conv_layers(dummy).flatten(1).shape[1]

        # 論文 Fig.3 に基づく 4層 FC
        self.fc_layers = nn.Sequential(
            nn.Linear(conv_out_dim, 1792),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(1792, 100),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, prediction_steps * output_dim),
        )

    def forward(self, x):
        """
        Args:
            x: (batch_size, scan_history_length, input_dim)
               scan_history_length=1 の場合: (batch, 1, 1081)
        """
        batch_size = x.size(0)

        # 最新フレームのみ使用（history=1 を想定）
        # x: (batch, history, input_dim) -> 最後のフレームを取り出す
        x = x[:, -1, :]  # (batch, input_dim)

        # Conv1d 用に channel 次元を追加
        x = x.unsqueeze(1)  # (batch, 1, input_dim)

        # 畳み込み
        x = self.conv_layers(x)       # (batch, 64, L)
        x = x.flatten(1)              # (batch, conv_out_dim)

        # 全結合
        x = self.fc_layers(x)         # (batch, prediction_steps * output_dim)

        # reshape
        x = x.view(batch_size, self.prediction_steps, -1)  # (batch, steps, output_dim)
        return x


class Model:
    def __init__(
        self,
        path,
        prediction_steps=1,
        scan_history_length=1,
        input_dim=1081,
        dropout=0.0,
    ):
        """
        Args:
            path: モデル重みファイルのパス（.pth）
            prediction_steps: 予測ステップ数（学習時と同じ値）
            scan_history_length: スキャン履歴長（学習時と同じ値）
            input_dim: スキャンデータの次元数
            dropout: ドロップアウト率
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.prediction_steps = prediction_steps
        self.scan_history_length = scan_history_length

        self.model = DNN(
            input_dim=input_dim,
            scan_history_length=scan_history_length,
            prediction_steps=prediction_steps,
            dropout=dropout,
        )

        checkpoint = torch.load(path, map_location=self.device, weights_only=True)
        self.model.load_state_dict(checkpoint["model_state_dict"])

        # 正規化統計量
        self.scan_mean   = checkpoint.get("scan_mean",   None)
        self.scan_std    = checkpoint.get("scan_std",    None)
        self.target_mean = checkpoint.get("target_mean", None)
        self.target_std  = checkpoint.get("target_std",  None)

        if self.scan_mean is not None:
            self.scan_mean = self.scan_mean.to(self.device)
            self.scan_std  = self.scan_std.to(self.device)
        if self.target_mean is not None:
            self.target_mean = self.target_mean.to(self.device)
            self.target_std  = self.target_std.to(self.device)

        self.model.to(self.device)
        self.model.eval()

        print(f"Model loaded from {path}")
        print(f"Normalization stats loaded: scan={self.scan_mean is not None}, "
              f"target={self.target_mean is not None}")

    def _normalize_scan(self, x):
        if self.scan_mean is not None:
            return (x - self.scan_mean) / (self.scan_std + 1e-8)
        return x

    def _denormalize_target(self, x):
        if self.target_mean is not None:
            return x * self.target_std + self.target_mean
        return x

    def inference(self, scan_ranges: np.ndarray) -> list:
        """
        Args:
            scan_ranges: (scan_history_length, input_dim) の numpy 配列
        Returns:
            prediction_steps=1 -> [throttle, angle]
            prediction_steps>1 -> [[t1,a1], [t2,a2], ...]
        """
        inp = torch.from_numpy(np.array(scan_ranges, dtype=np.float32)).unsqueeze(0)
        inp = inp.to(self.device)
        inp = self._normalize_scan(inp)

        with torch.no_grad():
            out = self.model(inp)
            out = self._denormalize_target(out)

        preds = out.cpu().numpy().squeeze(0)  # (prediction_steps, 2)

        if self.prediction_steps == 1:
            return preds.flatten().tolist()
        return preds.tolist()

    def inference_next_step_only(self, scan_ranges: np.ndarray) -> list:
        preds = self.inference(scan_ranges)
        if self.prediction_steps == 1:
            return preds
        return preds[0]
    

