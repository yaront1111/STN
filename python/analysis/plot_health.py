#!/usr/bin/env python3
"""Plot TRN health metrics from adaptive TRN logging"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load health log
log = pd.read_csv("data/trn_health.csv")

# Create figure with subplots
fig, axes = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle('TRN Health Diagnostics', fontsize=14, fontweight='bold')

# 1. NIS over time
ax = axes[0, 0]
accepted = log[log['trn'] == 1]
rejected = log[log['trn'] == 0]
ax.scatter(accepted['t'], accepted['nis'], c='green', s=10, alpha=0.6, label='Accepted')
ax.scatter(rejected['t'], rejected['nis'], c='red', s=10, alpha=0.6, label='Rejected')
ax.axhline(7.81, ls='--', c='orange', label='95% gate')
ax.set_xlabel('Time (s)')
ax.set_ylabel('NIS')
ax.set_title('Normalized Innovation Squared')
ax.legend()
ax.grid(True, alpha=0.3)

# 2. Alpha (soft update factor) over time
ax = axes[0, 1]
ax.scatter(accepted['t'], accepted['alpha'], c='blue', s=10, alpha=0.6)
ax.axhline(0.12, ls='--', c='gray', alpha=0.5, label='alpha_min')
ax.axhline(0.35, ls='--', c='gray', alpha=0.5, label='alpha_max')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Alpha')
ax.set_title('Adaptive Soft Update Factor')
ax.legend()
ax.grid(True, alpha=0.3)

# 3. Measurement variance R over time
ax = axes[1, 0]
ax.scatter(log['t'], log['R'], c='purple', s=10, alpha=0.6)
ax.set_xlabel('Time (s)')
ax.set_ylabel('R (m²)')
ax.set_title('Adaptive Measurement Variance')
ax.grid(True, alpha=0.3)

# 4. Terrain slope over time
ax = axes[1, 1]
ax.scatter(log['t'], log['slope'], c='brown', s=10, alpha=0.6)
ax.axhline(0.02, ls='--', c='red', alpha=0.5, label='slope_floor')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Slope (m/m)')
ax.set_title('Terrain Gradient Magnitude')
ax.legend()
ax.grid(True, alpha=0.3)

# 5. Residual over time
ax = axes[2, 0]
ax.scatter(accepted['t'], accepted['residual'], c='green', s=10, alpha=0.6, label='Accepted')
ax.scatter(rejected['t'], rejected['residual'], c='red', s=10, alpha=0.6, label='Rejected')
ax.axhline(1.5, ls='--', c='orange', alpha=0.5, label='Huber threshold')
ax.axhline(-1.5, ls='--', c='orange', alpha=0.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Residual (m)')
ax.set_title('AGL Measurement Residual')
ax.legend()
ax.grid(True, alpha=0.3)

# 6. TRN acceptance timeline
ax = axes[2, 1]
window = 10  # 10 second windows
n_windows = int(np.ceil(log['t'].max() / window))
accept_rates = []
times = []
for i in range(n_windows):
    t_start = i * window
    t_end = (i + 1) * window
    window_data = log[(log['t'] >= t_start) & (log['t'] < t_end)]
    if len(window_data) > 0:
        accept_rate = window_data['trn'].mean() * 100
        accept_rates.append(accept_rate)
        times.append((t_start + t_end) / 2)

ax.bar(times, accept_rates, width=window*0.8, alpha=0.7, color='teal')
ax.axhline(75, ls='--', c='orange', alpha=0.5, label='Target 75%')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Acceptance Rate (%)')
ax.set_title(f'TRN Accept Rate ({window}s windows)')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('data/trn_health_plots.png', dpi=150)
plt.show()

# Print summary statistics
print("=" * 60)
print("TRN HEALTH SUMMARY")
print("-" * 60)
print(f"Total TRN attempts: {len(log)}")
print(f"Accepted: {log['trn'].sum()} ({log['trn'].mean()*100:.1f}%)")
print(f"Rejected: {(1-log['trn']).sum()} ({(1-log['trn']).mean()*100:.1f}%)")
print()
print(f"NIS statistics (accepted only):")
print(f"  Mean: {accepted['nis'].mean():.3f}")
print(f"  Max: {accepted['nis'].max():.3f}")
print(f"  % above gate: {(accepted['nis'] > 7.81).mean()*100:.1f}%")
print()
print(f"Alpha statistics:")
print(f"  Mean: {accepted['alpha'].mean():.3f}")
print(f"  Min: {accepted['alpha'].min():.3f}")
print(f"  Max: {accepted['alpha'].max():.3f}")
print()
print(f"Terrain slope statistics:")
print(f"  Mean: {log['slope'].mean():.3f}")
print(f"  Max: {log['slope'].max():.3f}")
print(f"  % below floor: {(log['slope'] < 0.02).mean()*100:.1f}%")
print("=" * 60)