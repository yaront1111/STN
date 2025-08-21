#!/usr/bin/env python3
"""Plot position errors over time"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def main():
    # Load data
    root = Path(__file__).resolve().parents[2]
    truth = pd.read_csv(root / "data/sim_truth.csv")
    est = pd.read_csv(root / "data/run_output.csv")
    
    # Align lengths
    N = min(len(truth), len(est))
    truth = truth.iloc[:N]
    est = est.iloc[:N]
    
    # Compute errors
    err_n = est['pn'] - truth['pn']
    err_e = est['pe'] - truth['pe']
    err_d = est['pd'] - truth['pd']
    err_2d = np.sqrt(err_n**2 + err_e**2)
    
    # Create figure with 3 subplots
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle('STN Navigation Errors', fontsize=14, fontweight='bold')
    
    # Plot 2D horizontal error
    axes[0].plot(est['t'], err_2d, 'b-', linewidth=1.5, label='2D Error')
    axes[0].axhline(y=15, color='r', linestyle='--', alpha=0.5, label='Grade A Threshold (15m)')
    axes[0].set_ylabel('2D Error (m)')
    axes[0].set_title(f'Horizontal Position Error (Final: {err_2d.iloc[-1]:.1f}m, CEP95: {np.percentile(err_2d, 95):.1f}m)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    
    # Plot North-East errors
    axes[1].plot(est['t'], err_n, 'g-', linewidth=1, label='North Error')
    axes[1].plot(est['t'], err_e, 'r-', linewidth=1, label='East Error')
    axes[1].set_ylabel('Error (m)')
    axes[1].set_title('North and East Errors')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    
    # Plot vertical error
    axes[2].plot(est['t'], err_d, 'm-', linewidth=1.5)
    axes[2].set_ylabel('Vertical Error (m)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title(f'Vertical Error (RMSE: {np.sqrt((err_d**2).mean()):.2f}m)')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()