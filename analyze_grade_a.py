#!/usr/bin/env python3
"""
Analyze Grade A navigation test results
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys
from pathlib import Path

def analyze_results(csv_file):
    """Analyze navigation performance from CSV results"""
    
    # Load data
    df = pd.read_csv(csv_file)
    
    # Calculate statistics
    print("\n" + "="*60)
    print("GRADE A PERFORMANCE ANALYSIS")
    print("="*60)
    
    # Overall performance
    final_error = df['PosError'].iloc[-1]
    max_error = df['PosError'].max()
    mean_error = df['PosError'].mean()
    rms_error = np.sqrt((df['PosError']**2).mean())
    
    print(f"\nPOSITION ACCURACY:")
    print(f"  Final Error:     {final_error:8.1f} m")
    print(f"  Maximum Error:   {max_error:8.1f} m")
    print(f"  Mean Error:      {mean_error:8.1f} m")
    print(f"  RMS Error:       {rms_error:8.1f} m")
    
    # Velocity accuracy
    if 'VelError' in df.columns:
        vel_rms = np.sqrt((df['VelError']**2).mean())
        print(f"\nVELOCITY ACCURACY:")
        print(f"  RMS Error:       {vel_rms:8.2f} m/s")
    
    # Attitude accuracy  
    if 'AttError' in df.columns:
        att_rms = np.sqrt((df['AttError']**2).mean())
        print(f"\nATTITUDE ACCURACY:")
        print(f"  RMS Error:       {att_rms:8.2f} degrees")
    
    # Map matching performance
    if 'UpdateType' in df.columns:
        total_matches = (df['UpdateType'] == 'map_match').sum()
        match_rate = total_matches / (df['Time'].iloc[-1]) if df['Time'].iloc[-1] > 0 else 0
        print(f"\nMAP MATCHING:")
        print(f"  Total Matches:   {total_matches:.0f}")
        print(f"  Match Rate:      {match_rate:.2f} Hz")
    
    # Grading
    print(f"\n" + "="*60)
    print("GRADE ASSESSMENT:")
    print("="*60)
    
    grade = "F"
    if rms_error < 50:
        grade = "A"
        print(f"  Grade: A (Excellent) - RMS < 50m")
    elif rms_error < 100:
        grade = "B"
        print(f"  Grade: B (Good) - RMS < 100m")
    elif rms_error < 500:
        grade = "C"
        print(f"  Grade: C (Acceptable) - RMS < 500m")
    elif rms_error < 1000:
        grade = "D"
        print(f"  Grade: D (Poor) - RMS < 1000m")
    else:
        print(f"  Grade: F (Fail) - RMS > 1000m")
    
    print(f"\n  Target: Grade A requires RMS < 50m")
    print(f"  Current: {rms_error:.1f}m ({rms_error/50:.1f}x target)")
    
    # Performance vs time
    print(f"\nPERFORMANCE OVER TIME:")
    for t in [10, 20, 30, 60]:
        if len(df[df['Time'] <= t]) > 0:
            error_at_t = df[df['Time'] <= t]['PosError'].iloc[-1]
            print(f"  Error at {t:2d}s:    {error_at_t:8.1f} m")
    
    # Issues detected
    print(f"\n" + "="*60)
    print("ISSUES DETECTED:")
    print("="*60)
    
    issues = []
    if 'UpdateType' in df.columns and (df['UpdateType'] == 'map_match').sum() == 0:
        issues.append("No successful map matches - correlation failing")
    
    if rms_error > 1000:
        issues.append("Excessive drift - observability problem")
    
    if 'VelError' in df.columns and vel_rms > 10:
        issues.append("High velocity errors - IMU integration issues")
    
    if len(issues) == 0:
        print("  No critical issues detected")
    else:
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")
    
    # Recommendations
    print(f"\n" + "="*60)
    print("RECOMMENDATIONS:")
    print("="*60)
    
    if grade != "A":
        print("  Priority fixes for Grade A:")
        if 'UpdateType' in df.columns and (df['UpdateType'] == 'map_match').sum() == 0:
            print("  1. Fix gravity map correlation (use real anomaly data)")
        if rms_error > 500:
            print("  2. Tune UKF process/measurement noise")
            print("  3. Add more sensor updates (magnetometer, barometer)")
        if rms_error > 100:
            print("  4. Optimize map matching parameters")
            print("  5. Implement adaptive filtering")
    else:
        print("  System achieving Grade A performance!")
    
    # Create plots
    print(f"\nGenerating plots...")
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # Position error over time
    axes[0,0].plot(df['Time'], df['PosError'])
    axes[0,0].axhline(y=50, color='g', linestyle='--', label='Grade A')
    axes[0,0].axhline(y=100, color='y', linestyle='--', label='Grade B')
    axes[0,0].axhline(y=500, color='r', linestyle='--', label='Grade C')
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Position Error (m)')
    axes[0,0].set_title('Position Error vs Time')
    axes[0,0].legend()
    axes[0,0].grid(True)
    
    # Trajectory - skip if columns don't exist
    if 'true_x' in df.columns and 'true_y' in df.columns:
        axes[0,1].plot(df['true_x'], df['true_y'], 'g-', label='True', linewidth=2)
        axes[0,1].plot(df['est_x'], df['est_y'], 'b--', label='Estimated', alpha=0.7)
        axes[0,1].set_xlabel('East (m)')
        axes[0,1].set_ylabel('North (m)')
        axes[0,1].set_title('2D Trajectory')
        axes[0,1].legend()
        axes[0,1].grid(True)
        axes[0,1].axis('equal')
    else:
        axes[0,1].text(0.5, 0.5, 'Trajectory data not available', 
                      ha='center', va='center', transform=axes[0,1].transAxes)
        axes[0,1].set_title('2D Trajectory')
    
    # Velocity error
    if 'VelError' in df.columns:
        axes[1,0].plot(df['Time'], df['VelError'])
        axes[1,0].axhline(y=0.5, color='g', linestyle='--', label='Grade A')
        axes[1,0].set_xlabel('Time (s)')
        axes[1,0].set_ylabel('Velocity Error (m/s)')
        axes[1,0].set_title('Velocity Error vs Time')
        axes[1,0].legend()
        axes[1,0].grid(True)
    
    # Map matches or update types
    if 'UpdateType' in df.columns:
        map_match_count = (df['UpdateType'] == 'map_match').cumsum()
        axes[1,1].plot(df['Time'], map_match_count)
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Cumulative Map Matches')
        axes[1,1].set_title('Map Matching Success')
        axes[1,1].grid(True)
    elif 'map_matches' in df.columns:
        axes[1,1].plot(df['Time'], df['map_matches'])
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Cumulative Map Matches')
        axes[1,1].set_title('Map Matching Success')
        axes[1,1].grid(True)
    
    plt.tight_layout()
    output_file = csv_file.replace('.csv', '_analysis.png')
    plt.savefig(output_file)
    print(f"  Saved plots to: {output_file}")
    
    return grade, rms_error

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_grade_a.py <results.csv>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    if not Path(csv_file).exists():
        print(f"Error: File {csv_file} not found")
        sys.exit(1)
    
    grade, rms_error = analyze_results(csv_file)
    
    print(f"\n" + "="*60)
    print(f"FINAL GRADE: {grade}")
    print("="*60)