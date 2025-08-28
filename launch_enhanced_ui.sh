#!/bin/bash

# STN Navigation System - Enhanced UI Launcher
# Complete testing and monitoring interface

echo "========================================"
echo "  STN Navigation System - Enhanced UI"
echo "========================================"
echo

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check Python 3
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python 3 is not installed${NC}"
    echo "Please install Python 3 to continue."
    exit 1
fi

echo -e "${BLUE}🔍 Checking system requirements...${NC}"

# Check and install required packages
PACKAGES="flask pandas numpy matplotlib plotly scipy"
MISSING_PACKAGES=""

for package in $PACKAGES; do
    if ! python3 -c "import $package" 2>/dev/null; then
        MISSING_PACKAGES="$MISSING_PACKAGES $package"
    fi
done

if [ ! -z "$MISSING_PACKAGES" ]; then
    echo -e "${YELLOW}📦 Installing required packages:$MISSING_PACKAGES${NC}"
    
    if command -v pip3 &> /dev/null; then
        pip3 install $MISSING_PACKAGES
    elif command -v pip &> /dev/null; then
        pip install $MISSING_PACKAGES
    else
        echo -e "${RED}❌ pip is not installed${NC}"
        echo "Please install pip or run: pip install -r requirements.txt"
        exit 1
    fi
fi

# Check if build exists, if not build it
if [ ! -f "build/stn_demo" ]; then
    echo -e "${YELLOW}🔨 Building C++ components for first time use...${NC}"
    mkdir -p build
    cd build
    
    # Configure with all features
    cmake .. -DBUILD_TESTS=ON -DBUILD_BENCHMARKS=ON > /dev/null 2>&1
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ CMake configuration failed${NC}"
        echo "Please check that CMake and Eigen3 are installed:"
        echo "  Ubuntu/Debian: sudo apt-get install cmake libeigen3-dev"
        echo "  macOS: brew install cmake eigen"
        exit 1
    fi
    
    # Build with parallel jobs
    make -j4 > /dev/null 2>&1
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}❌ Build failed${NC}"
        echo "Please check the build errors above"
        exit 1
    fi
    
    cd ..
    echo -e "${GREEN}✅ Build complete${NC}"
fi

# Check which UI to launch
if [ "$1" == "simple" ]; then
    echo -e "${BLUE}🚀 Launching Simple UI...${NC}"
    UI_SCRIPT="simple_ui.py"
    PORT=5000
else
    echo -e "${BLUE}🚀 Launching Enhanced UI with full test suite...${NC}"
    UI_SCRIPT="enhanced_ui.py"
    PORT=5001
fi

# Check if UI script exists
if [ ! -f "$UI_SCRIPT" ]; then
    echo -e "${RED}❌ UI script not found: $UI_SCRIPT${NC}"
    exit 1
fi

echo
echo "========================================"
echo -e "${GREEN}✅ System ready!${NC}"
echo
echo -e "📱 Open your browser to: ${BLUE}http://localhost:$PORT${NC}"
echo
echo "Features available:"
echo "  ✓ Complete navigation pipeline"
echo "  ✓ Unit tests and benchmarks"
echo "  ✓ Monte Carlo simulation"
echo "  ✓ Real-time monitoring"
echo "  ✓ Configuration management"
echo "  ✓ Performance grading"
echo
echo -e "${YELLOW}Press Ctrl+C to stop the server${NC}"
echo "========================================"
echo

# Launch the UI
python3 $UI_SCRIPT