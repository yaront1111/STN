.PHONY: build debug release clean test run format lint docker help

# Default build directory
BUILD_DIR ?= build
BUILD_TYPE ?= Release

help: ## Show this help message
	@echo "AION Build System"
	@echo "=================="
	@echo ""
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  %-15s %s\n", $$1, $$2}'

build: ## Build in Release mode (default)
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) ..
	@cmake --build $(BUILD_DIR) -j$(shell nproc)

debug: ## Build in Debug mode
	@mkdir -p build-debug
	@cd build-debug && cmake -DCMAKE_BUILD_TYPE=Debug ..
	@cmake --build build-debug -j$(shell nproc)

release: ## Build in Release mode with optimizations
	@mkdir -p build-release
	@cd build-release && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-march=native" ..
	@cmake --build build-release -j$(shell nproc)

clean: ## Clean build artifacts
	@rm -rf build build-* logs/*.bag

test: build ## Run unit tests
	@cd $(BUILD_DIR) && ctest --output-on-failure

run: build ## Run AION node with default config
	@./$(BUILD_DIR)/apps/aion_node --config configs/aion_default.yaml

format: ## Format code using clang-format
	@find . -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | grep -v build | xargs clang-format -i

lint: ## Run static analysis
	@pre-commit run --all-files

docker: ## Build Docker image
	@docker build -f docker/Dockerfile.runtime -t aion:latest .

docker-dev: ## Build development Docker image
	@docker build -f docker/Dockerfile.dev -t aion:dev .

install: build ## Install to system
	@cd $(BUILD_DIR) && sudo make install

# Development helpers
setup-dev: ## Setup development environment
	@pip3 install --user pre-commit pyyaml numpy
	@pre-commit install
	@echo "Development environment ready!"

generate-compile-commands: ## Generate compile_commands.json for IDE
	@mkdir -p build
	@cd build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
	@ln -sf build/compile_commands.json .

# Data management
download-dem: ## Download SRTM DEM data
	@mkdir -p data/dem
	@scripts/datasets/fetch_srtm.sh

# Calibration
calibrate-imu: ## Run IMU calibration
	@python3 scripts/calib/calib_imu.py

calibrate-camera: ## Run camera calibration
	@python3 scripts/calib/calib_camera.py