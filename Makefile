.PHONY: clean build cache-clean run help

# デフォルトターゲット
.DEFAULT_GOAL := help

# ディレクトリのクリーンアップ
clean:
	@echo "Cleaning install, build, and log directories..."
	rm -rf install build log
	@echo "Clean completed."

# 通常のビルド
build:
	@echo "Building with colcon..."
	colcon build --symlink-install

# キャッシュクリーンしてビルド
cache-clean:
	@echo "Cleaning cache and building..."
	colcon build --cmake-clean-cache

# セットアップとlaunchファイルの実行
run:
	@echo "Sourcing setup.bash and launching autonomous_driving..."
	bash -c "source install/setup.bash && ros2 launch launch/autonomous_driving.py"

# ヘルプ
help:
	@echo "Available targets:"
	@echo "  make clean         - Remove install, build, and log directories"
	@echo "  make build         - Build with colcon (symlink-install)"
	@echo "  make cache-clean   - Clean CMake cache and build"
	@echo "  make run           - Source setup and launch autonomous_driving.py"
	@echo "  make help          - Show this help message"