#!/bin/bash

echo "========================================"
echo "     STN Control Panel Launcher"
echo "========================================"
echo

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed. Please install Python 3."
    exit 1
fi

# Check if Flask is installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo "📦 Installing required packages..."
    echo "This may take a minute..."
    
    # Try pip3 first, then pip
    if command -v pip3 &> /dev/null; then
        pip3 install flask pandas numpy matplotlib plotly
    elif command -v pip &> /dev/null; then
        pip install flask pandas numpy matplotlib plotly
    else
        echo "❌ pip is not installed. Please install pip first."
        echo "   Or manually run: pip install -r requirements.txt"
        exit 1
    fi
fi

# Check if build exists
if [ ! -f "build/stn_demo" ]; then
    echo "🔨 Building C++ code for first time use..."
    mkdir -p build
    cd build
    cmake .. > /dev/null 2>&1
    make > /dev/null 2>&1
    cd ..
    echo "✅ Build complete"
fi

# Launch the UI
echo
echo "🚀 Launching STN Control Panel..."
echo "📱 Open your browser to: http://localhost:5000"
echo
echo "Press Ctrl+C to stop the server"
echo "========================================"
echo

python3 simple_ui.py