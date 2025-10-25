#!/bin/bash
# CHIMERA PX4 SITL Setup Script
# Sets up PX4 Software-In-The-Loop simulation for testing CHIMERA navigation
#
# Usage:
#   ./setup_px4_sitl.sh [install|run|clean]
#
# Commands:
#   install - Clone and build PX4-Autopilot
#   run     - Launch PX4 SITL with iris quadcopter
#   clean   - Clean PX4 build artifacts

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
PX4_DIR="${HOME}/PX4-Autopilot"
PX4_BRANCH="v1.14.0"  # Stable release
CHIMERA_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

# Helper functions
info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    exit 1
}

# Check if PX4-Autopilot is installed
check_px4_installation() {
    if [ ! -d "$PX4_DIR" ]; then
        return 1
    fi
    return 0
}

# Install PX4-Autopilot
install_px4() {
    info "Installing PX4-Autopilot..."

    # Check if already installed
    if check_px4_installation; then
        warn "PX4-Autopilot already exists at $PX4_DIR"
        read -p "Reinstall? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            info "Skipping installation"
            return 0
        fi
        rm -rf "$PX4_DIR"
    fi

    # Clone PX4-Autopilot
    info "Cloning PX4-Autopilot $PX4_BRANCH..."
    git clone https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR" --recursive
    cd "$PX4_DIR"
    git checkout "$PX4_BRANCH"
    git submodule update --init --recursive

    # Install dependencies (Ubuntu)
    info "Installing PX4 dependencies..."
    bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

    # Build for SITL
    info "Building PX4 for SITL..."
    make px4_sitl_default

    info "PX4-Autopilot installation complete!"
    info "Location: $PX4_DIR"
}

# Run PX4 SITL
run_px4_sitl() {
    if ! check_px4_installation; then
        error "PX4-Autopilot not found. Run './setup_px4_sitl.sh install' first."
    fi

    info "Starting PX4 SITL with iris quadcopter..."
    cd "$PX4_DIR"

    # Set environment variables for MAVROS/MAVLINK
    export PX4_SIM_SPEED_FACTOR=1.0

    # Launch SITL
    # This will start:
    # - PX4 SITL on UDP port 14540 (mavlink)
    # - Gazebo simulation with iris quadcopter
    info "PX4 SITL will be available on:"
    info "  - MAVLink: UDP 14540 (for MAVROS)"
    info "  - Gazebo: http://localhost:11345"
    info ""
    info "Press Ctrl+C to stop SITL"
    echo ""

    # Run with iris (basic quadcopter with IMU, mag, baro)
    make px4_sitl gazebo-classic_iris

    # Note: For advanced sensors (lidar, optical flow), use:
    # make px4_sitl gazebo-classic_iris_opt_flow
}

# Clean PX4 build artifacts
clean_px4() {
    if ! check_px4_installation; then
        warn "PX4-Autopilot not found at $PX4_DIR"
        return 0
    fi

    info "Cleaning PX4 build artifacts..."
    cd "$PX4_DIR"
    make clean
    make distclean

    info "PX4 build artifacts cleaned"
}

# Show usage
show_usage() {
    cat << EOF
CHIMERA PX4 SITL Setup Script

Usage:
  $0 [command]

Commands:
  install    Clone and build PX4-Autopilot
  run        Launch PX4 SITL with iris quadcopter
  clean      Clean PX4 build artifacts
  help       Show this help message

Examples:
  # Install PX4-Autopilot
  $0 install

  # Run SITL simulation
  $0 run

  # In another terminal, launch CHIMERA ROS2 node with MAVROS
  ros2 launch chimera_nav chimera_sitl.launch.py

Notes:
  - PX4 will be installed to: $PX4_DIR
  - SITL listens on UDP port 14540 for MAVLink
  - Gazebo GUI available at http://localhost:11345
  - Use 'pxh> commander takeoff' in PX4 console to takeoff

For more information:
  https://docs.px4.io/main/en/simulation/
EOF
}

# Main script
main() {
    local command="${1:-help}"

    case "$command" in
        install)
            install_px4
            ;;
        run)
            run_px4_sitl
            ;;
        clean)
            clean_px4
            ;;
        help|--help|-h)
            show_usage
            ;;
        *)
            error "Unknown command: $command. Use 'help' for usage information."
            ;;
    esac
}

main "$@"
