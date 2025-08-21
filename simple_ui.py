#!/usr/bin/env python3
"""
Simple STN Control Panel - Run everything with one click
"""

from flask import Flask, render_template_string, jsonify
import subprocess
import os
from pathlib import Path

app = Flask(__name__)
base_path = Path(__file__).parent

HTML = '''
<!DOCTYPE html>
<html>
<head>
    <title>STN Control Panel</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 800px;
            margin: 50px auto;
            padding: 20px;
            background: #f5f5f5;
        }
        h1 {
            color: #333;
            border-bottom: 3px solid #4CAF50;
            padding-bottom: 10px;
        }
        .section {
            background: white;
            border-radius: 8px;
            padding: 20px;
            margin: 20px 0;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        button {
            background: #4CAF50;
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 16px;
            margin: 10px 10px 10px 0;
        }
        button:hover {
            background: #45a049;
        }
        button:disabled {
            background: #ccc;
            cursor: not-allowed;
        }
        .explanation {
            color: #666;
            font-size: 14px;
            margin-left: 20px;
            margin-bottom: 10px;
            font-style: italic;
        }
        .console {
            background: #1e1e1e;
            color: #0f0;
            padding: 15px;
            border-radius: 4px;
            font-family: monospace;
            height: 300px;
            overflow-y: auto;
            margin-top: 20px;
            white-space: pre-wrap;
        }
        .status {
            padding: 10px;
            border-radius: 4px;
            margin: 10px 0;
            display: none;
        }
        .status.success {
            background: #d4edda;
            color: #155724;
            display: block;
        }
        .status.error {
            background: #f8d7da;
            color: #721c24;
            display: block;
        }
        .status.info {
            background: #d1ecf1;
            color: #0c5460;
            display: block;
        }
        h2 {
            color: #4CAF50;
            margin-top: 0;
        }
        .step-number {
            background: #4CAF50;
            color: white;
            border-radius: 50%;
            width: 30px;
            height: 30px;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            margin-right: 10px;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <h1>🚀 STN Space-Time Navigation Control Panel</h1>
    
    <div class="section">
        <h2><span class="step-number">1</span>Generate Test Data</h2>
        
        <button onclick="runAction('simulate')">Generate Flight Path</button>
        <div class="explanation">→ Creates a simulated aircraft trajectory with IMU sensor data (accelerometer + gyroscope)</div>
        
        <button onclick="runAction('radalt')">Add Radar Altimeter</button>
        <div class="explanation">→ Adds terrain height measurements (like a radar looking down at the ground)</div>
        
        <div id="status_simulate" class="status"></div>
    </div>
    
    <div class="section">
        <h2><span class="step-number">2</span>Run Navigation</h2>
        
        <button onclick="runAction('build')">Build Navigation Code</button>
        <div class="explanation">→ Compiles the C++ navigation algorithms (only needed once)</div>
        
        <button onclick="runAction('navigate')">Run Navigation System</button>
        <div class="explanation">→ Processes sensor data through INS + Terrain Matching + Gravity fusion</div>
        
        <div id="status_navigate" class="status"></div>
    </div>
    
    <div class="section">
        <h2><span class="step-number">3</span>Check Results</h2>
        
        <button onclick="runAction('grade')">Grade Performance</button>
        <div class="explanation">→ Calculates accuracy metrics and assigns a grade (A, B, C)</div>
        
        <button onclick="runAction('plot')">Show 3D Trajectory</button>
        <div class="explanation">→ Opens interactive 3D plot comparing true vs estimated flight path</div>
        
        <button onclick="runAction('plot2d')">Show 2D Plots</button>
        <div class="explanation">→ Shows position errors over time</div>
        
        <button onclick="showWebPlot()">Show Trajectory Here</button>
        <div class="explanation">→ Shows the trajectory right in this page</div>
        
        <div id="status_results" class="status"></div>
    </div>
    
    <div class="section" id="plot_section" style="display: none;">
        <h2>📍 Navigation Trajectory</h2>
        <canvas id="trajectory_canvas" width="700" height="400" style="border: 1px solid #ddd; border-radius: 4px; background: white;"></canvas>
        <div style="margin-top: 10px;">
            <span style="color: blue;">● Truth Path</span> &nbsp;&nbsp;
            <span style="color: red;">● Estimated Path</span> &nbsp;&nbsp;
            <span style="color: green;">● Start</span> &nbsp;&nbsp;
            <span style="color: orange;">● End</span>
        </div>
    </div>
    
    <div class="section">
        <h2>Quick Actions</h2>
        
        <button onclick="runAll()" style="background: #ff9800;">🎯 Run Everything</button>
        <div class="explanation">→ Runs the complete pipeline: Generate → Navigate → Grade</div>
        
        <button onclick="clearConsole()" style="background: #666;">Clear Output</button>
        <div class="explanation">→ Clears the console below</div>
    </div>
    
    <div class="console" id="console">Ready. Click any button to start.

What this system does:
- Navigates without GPS using terrain shape and gravity
- Combines INS (accelerometer + gyro) with terrain matching
- Achieves ~11 meter accuracy (Grade A-)</div>
    
    <script>
        function updateConsole(text) {
            const console = document.getElementById('console');
            console.textContent += '\\n' + text;
            console.scrollTop = console.scrollHeight;
        }
        
        function clearConsole() {
            document.getElementById('console').textContent = 'Console cleared.\\n';
        }
        
        async function runAction(action) {
            // Clear previous status
            document.querySelectorAll('.status').forEach(s => s.style.display = 'none');
            
            updateConsole(`\\n>>> Running: ${action}...`);
            
            try {
                const response = await fetch(`/run/${action}`, {method: 'POST'});
                const result = await response.json();
                
                updateConsole(result.output);
                
                // Show status
                let statusDiv;
                if (action === 'simulate' || action === 'radalt') {
                    statusDiv = document.getElementById('status_simulate');
                } else if (action === 'build' || action === 'navigate') {
                    statusDiv = document.getElementById('status_navigate');
                } else {
                    statusDiv = document.getElementById('status_results');
                }
                
                statusDiv.textContent = result.message;
                statusDiv.className = 'status ' + (result.success ? 'success' : 'error');
                
            } catch (error) {
                updateConsole(`Error: ${error}`);
            }
        }
        
        async function runAll() {
            updateConsole('\\n🚀 RUNNING COMPLETE PIPELINE...\\n');
            
            // Run in sequence
            await runAction('simulate');
            await new Promise(r => setTimeout(r, 1000));
            
            await runAction('radalt');
            await new Promise(r => setTimeout(r, 1000));
            
            await runAction('build');
            await new Promise(r => setTimeout(r, 1000));
            
            await runAction('navigate');
            await new Promise(r => setTimeout(r, 1000));
            
            await runAction('grade');
            
            updateConsole('\\n✅ PIPELINE COMPLETE! Check the grade above.');
            
            // Auto-show plot
            setTimeout(showWebPlot, 500);
        }
        
        async function showWebPlot() {
            // Fetch trajectory data
            const response = await fetch('/api/trajectory_data');
            const data = await response.json();
            
            if (!data.success) {
                updateConsole('No trajectory data available. Run navigation first.');
                return;
            }
            
            // Show plot section
            document.getElementById('plot_section').style.display = 'block';
            
            // Get canvas and context
            const canvas = document.getElementById('trajectory_canvas');
            const ctx = canvas.getContext('2d');
            const width = canvas.width;
            const height = canvas.height;
            
            // Clear canvas
            ctx.clearRect(0, 0, width, height);
            
            // Find data bounds
            const allX = [...data.truth_e, ...data.est_e];
            const allY = [...data.truth_n, ...data.est_n];
            const minX = Math.min(...allX);
            const maxX = Math.max(...allX);
            const minY = Math.min(...allY);
            const maxY = Math.max(...allY);
            const rangeX = maxX - minX;
            const rangeY = maxY - minY;
            
            // Scale to canvas with padding
            const padding = 40;
            const scaleX = (width - 2*padding) / rangeX;
            const scaleY = (height - 2*padding) / rangeY;
            const scale = Math.min(scaleX, scaleY);
            
            function toCanvas(x, y) {
                return {
                    x: padding + (x - minX) * scale,
                    y: height - padding - (y - minY) * scale
                };
            }
            
            // Draw grid
            ctx.strokeStyle = '#e0e0e0';
            ctx.lineWidth = 0.5;
            for (let i = 0; i <= 10; i++) {
                const x = padding + i * (width - 2*padding) / 10;
                const y = padding + i * (height - 2*padding) / 10;
                ctx.beginPath();
                ctx.moveTo(x, padding);
                ctx.lineTo(x, height - padding);
                ctx.stroke();
                ctx.beginPath();
                ctx.moveTo(padding, y);
                ctx.lineTo(width - padding, y);
                ctx.stroke();
            }
            
            // Draw truth trajectory
            ctx.strokeStyle = 'blue';
            ctx.lineWidth = 2;
            ctx.beginPath();
            for (let i = 0; i < data.truth_e.length; i++) {
                const pt = toCanvas(data.truth_e[i], data.truth_n[i]);
                if (i === 0) ctx.moveTo(pt.x, pt.y);
                else ctx.lineTo(pt.x, pt.y);
            }
            ctx.stroke();
            
            // Draw estimated trajectory
            ctx.strokeStyle = 'red';
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            for (let i = 0; i < data.est_e.length; i++) {
                const pt = toCanvas(data.est_e[i], data.est_n[i]);
                if (i === 0) ctx.moveTo(pt.x, pt.y);
                else ctx.lineTo(pt.x, pt.y);
            }
            ctx.stroke();
            
            // Draw start and end points
            // Start point (green)
            const start = toCanvas(data.truth_e[0], data.truth_n[0]);
            ctx.fillStyle = 'green';
            ctx.beginPath();
            ctx.arc(start.x, start.y, 5, 0, 2*Math.PI);
            ctx.fill();
            
            // End points (orange)
            const endTruth = toCanvas(data.truth_e[data.truth_e.length-1], data.truth_n[data.truth_n.length-1]);
            const endEst = toCanvas(data.est_e[data.est_e.length-1], data.est_n[data.est_n.length-1]);
            ctx.fillStyle = 'orange';
            ctx.beginPath();
            ctx.arc(endTruth.x, endTruth.y, 5, 0, 2*Math.PI);
            ctx.fill();
            ctx.beginPath();
            ctx.arc(endEst.x, endEst.y, 5, 0, 2*Math.PI);
            ctx.fill();
            
            // Add scale info
            ctx.fillStyle = '#666';
            ctx.font = '12px Arial';
            ctx.fillText(`Scale: ${rangeX.toFixed(0)}m x ${rangeY.toFixed(0)}m`, padding, height - 10);
            ctx.fillText(`Final Error: ${data.final_error.toFixed(1)}m`, padding, 20);
            
            updateConsole('Trajectory plotted in page.');
        }
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/api/trajectory_data')
def get_trajectory_data():
    """Get trajectory data for plotting"""
    try:
        import pandas as pd
        import numpy as np
        
        # Read data files
        truth_file = base_path / 'data' / 'sim_truth.csv'
        est_file = base_path / 'data' / 'run_output.csv'
        
        if not truth_file.exists() or not est_file.exists():
            return jsonify({'success': False, 'message': 'Data files not found'})
        
        truth = pd.read_csv(truth_file)
        est = pd.read_csv(est_file)
        
        # Align lengths
        N = min(len(truth), len(est))
        truth = truth.iloc[:N]
        est = est.iloc[:N]
        
        # Calculate final error
        err_n = est['pn'].iloc[-1] - truth['pn'].iloc[-1]
        err_e = est['pe'].iloc[-1] - truth['pe'].iloc[-1]
        final_error = np.sqrt(err_n**2 + err_e**2)
        
        # Downsample for web display (every 10th point)
        step = max(1, len(truth) // 200)
        
        return jsonify({
            'success': True,
            'truth_n': truth['pn'].iloc[::step].tolist(),
            'truth_e': truth['pe'].iloc[::step].tolist(),
            'est_n': est['pn'].iloc[::step].tolist(),
            'est_e': est['pe'].iloc[::step].tolist(),
            'final_error': float(final_error)
        })
        
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)})

@app.route('/run/<action>', methods=['POST'])
def run_action(action):
    """Run specific action and return result"""
    
    try:
        if action == 'simulate':
            # Generate simulated IMU data
            cmd = ['python3', 'python/sim/run_sim.py']
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(base_path))
            
            return jsonify({
                'success': result.returncode == 0,
                'output': result.stdout if result.stdout else "Generated flight trajectory with IMU data",
                'message': "✓ Flight path generated (120 seconds, 50 m/s)"
            })
            
        elif action == 'radalt':
            # Add radar altimeter measurements
            cmd = ['python3', 'python/sim/add_radalt.py']
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(base_path))
            
            return jsonify({
                'success': result.returncode == 0,
                'output': result.stdout if result.stdout else "Added radar altimeter data",
                'message': "✓ Radar altimeter data added"
            })
            
        elif action == 'build':
            # Build C++ code
            build_path = base_path / 'build'
            build_path.mkdir(exist_ok=True)
            
            # Run cmake
            cmd = ['cmake', '..']
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(build_path))
            
            if result.returncode == 0:
                # Build executable
                cmd = ['cmake', '--build', '.', '--target', 'stn_demo']
                result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(build_path))
            
            return jsonify({
                'success': result.returncode == 0,
                'output': "Build output:\n" + result.stdout[-500:],  # Last 500 chars
                'message': "✓ Navigation code compiled successfully" if result.returncode == 0 else "✗ Build failed"
            })
            
        elif action == 'navigate':
            # Run navigation
            exe = base_path / 'build' / 'stn_demo'
            if not exe.exists():
                return jsonify({
                    'success': False,
                    'output': "Navigation executable not found. Please build first.",
                    'message': "✗ Please build the code first"
                })
            
            # Check if input files exist
            imu_file = base_path / 'data' / 'sim_imu.csv'
            radalt_file = base_path / 'data' / 'radalt.csv'
            
            if not imu_file.exists():
                return jsonify({
                    'success': False,
                    'output': "IMU data not found. Please generate flight path first.",
                    'message': "✗ Generate flight path first"
                })
            
            if not radalt_file.exists():
                return jsonify({
                    'success': False,
                    'output': "Radar altimeter data not found. Please add radar altimeter first.",
                    'message': "✗ Add radar altimeter first"
                })
            
            cmd = [str(exe), 'data/sim_imu.csv', 'data/run_output.csv', 'data/radalt.csv']
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(base_path))
            
            # Extract key info from output
            output_lines = result.stdout.split('\n')
            summary = '\n'.join([line for line in output_lines if 'TRN' in line or 'Wrote' in line])
            
            return jsonify({
                'success': result.returncode == 0,
                'output': summary if summary else result.stdout,
                'message': "✓ Navigation complete (TRN + Gravity fusion active)"
            })
            
        elif action == 'grade':
            # Grade performance
            cmd = ['python3', 'python/analysis/grade_check.py']
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(base_path))
            
            # Extract grade from output
            grade = "Unknown"
            if "GRADE:" in result.stdout:
                for line in result.stdout.split('\n'):
                    if "GRADE:" in line:
                        grade = line.split("GRADE:")[1].strip()
                        break
            
            return jsonify({
                'success': result.returncode == 0,
                'output': result.stdout,
                'message': f"✓ Performance Grade: {grade}"
            })
            
        elif action == 'plot':
            # Launch 3D plot (runs in background)
            cmd = ['python3', 'python/visualization/plot_results.py']
            subprocess.Popen(cmd, cwd=str(base_path))
            
            return jsonify({
                'success': True,
                'output': "Opening 3D trajectory plot in browser...\nThis shows the true path (thick line) vs estimated path (thin line)",
                'message': "✓ 3D plot opened in new browser tab"
            })
            
        elif action == 'plot2d':
            # Launch 2D error plots
            cmd = ['python3', 'python/visualization/plot_errors.py']
            subprocess.Popen(cmd, cwd=str(base_path))
            
            return jsonify({
                'success': True,
                'output': "Opening error plots...\nShows position errors over time",
                'message': "✓ Error plots opened"
            })
            
        else:
            return jsonify({
                'success': False,
                'output': f"Unknown action: {action}",
                'message': "✗ Unknown action"
            })
            
    except Exception as e:
        return jsonify({
            'success': False,
            'output': str(e),
            'message': f"✗ Error: {str(e)}"
        })

if __name__ == '__main__':
    print("\n" + "="*50)
    print("STN Control Panel Starting...")
    print("="*50)
    print("\nOpen your browser to: http://localhost:5000\n")
    print("The UI will explain what each button does.\n")
    app.run(debug=False, port=5000, host='127.0.0.1')