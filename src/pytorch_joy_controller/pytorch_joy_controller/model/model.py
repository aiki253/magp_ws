import torch
import torch.nn as nn

# シードを固定
torch.manual_seed(42)


class DNN(nn.Module):
    def __init__(self, input_dim=1081, output_dim=2):
        super().__init__()
        self.conv1 = nn.Conv1d(1, 24, kernel_size=5, stride=2, padding=2)
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
        self.fc4 = nn.Linear(10, output_dim)

        self.relu = nn.ReLU()

    def forward(self, x):
        x = x.unsqueeze(1)
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.relu(self.conv3(x))
        x = self.relu(self.conv4(x))
        x = self.relu(self.conv5(x))
        x = x.flatten(1)
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.relu(self.fc3(x))
        return self.fc4(x)


if __name__ == "__main__":
    import torch.optim as optim
    import matplotlib.pyplot as plt
    from torch.utils.data import DataLoader
    from dataset import RosbagDataset, load_rosbag_data
    
    # デバイスの設定
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = DNN().to(device)  # モデルをGPUに転送
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    num_epochs = 100
    batch_size = 64

    bag_path = (
        "../dataset/rosbag2_2025_10_08-15_15_28/rosbag2_2025_10_08-15_15_28_0.db3"
    )
    scan_df, synchronized_joy_df = load_rosbag_data(bag_path)
    dataset = RosbagDataset(scan_df, synchronized_joy_df)

    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = torch.utils.data.random_split(
        dataset, [train_size, val_size]
    )

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    # train model
    train_losses = []
    val_losses = []
    best_val_loss = float("inf")
    patience = 10
    trigger_times = 0

    for epoch in range(num_epochs):
        model.train()
        running_train_loss = 0.0
        for scan_ranges, joy_speeds, joy_angles in train_loader:
            # データをGPUに転送
            scan_ranges = scan_ranges.to(device)
            joy_speeds = joy_speeds.to(device)
            joy_angles = joy_angles.to(device)

            optimizer.zero_grad()
            outputs = model(scan_ranges)
            targets = torch.stack((joy_speeds, joy_angles), dim=1)
            loss = criterion(outputs, targets)
            loss.backward()
            optimizer.step()
            running_train_loss += loss.item() * scan_ranges.size(0)
        epoch_train_loss = running_train_loss / len(train_loader.dataset)
        train_losses.append(epoch_train_loss)

        model.eval()
        running_val_loss = 0.0
        with torch.no_grad():
            for scan_ranges, joy_speeds, joy_angles in val_loader:
                # データをGPUに転送
                scan_ranges = scan_ranges.to(device)
                joy_speeds = joy_speeds.to(device)
                joy_angles = joy_angles.to(device)

                outputs = model(scan_ranges)
                targets = torch.stack((joy_speeds, joy_angles), dim=1)
                loss = criterion(outputs, targets)
                running_val_loss += loss.item() * scan_ranges.size(0)
        epoch_val_loss = running_val_loss / len(val_loader.dataset)
        val_losses.append(epoch_val_loss)

        print(
            f"Epoch {epoch + 1}/{num_epochs}, Train Loss: {epoch_train_loss:.4f}, Val Loss: {epoch_val_loss:.4f}"
        )

        if epoch_val_loss < best_val_loss:
            best_val_loss = epoch_val_loss
            trigger_times = 0
        else:
            trigger_times += 1
            if trigger_times >= patience:
                print("Early stopping!")
                break

    # 損失のプロット
    plt.plot(train_losses, label="Train Loss")
    plt.plot(val_losses, label="Validation Loss")
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.legend()
    plt.savefig("../figures/training_loss.png")
    plt.show()

    # モデルをCPUに移動してから保存
    model.eval()
    model = model.cpu()

    # pthファイルとして保存（モデル全体を保存）
    torch.save(model.state_dict(), "../model/model.pth")
    print("Model saved to ../model/model.pth")
