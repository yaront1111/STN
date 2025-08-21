#!/usr/bin/env python3
"""Analyze TRN health metrics from adaptive TRN logging"""

import pandas as pd
import numpy as np

# Load health log
log = pd.read_csv("data/trn_health.csv")
accepted = log[log['trn'] == 1]
rejected = log[log['trn'] == 0]

print("=" * 70)
print("TRN HEALTH ANALYSIS")
print("=" * 70)

print(f"\n📊 ACCEPTANCE STATISTICS")
print("-" * 40)
print(f"Total attempts:     {len(log):8d}")
print(f"Accepted:           {log['trn'].sum():8d} ({log['trn'].mean()*100:5.1f}%)")
print(f"Rejected:           {(1-log['trn']).sum():8d} ({(1-log['trn']).mean()*100:5.1f}%)")

print(f"\n📈 NIS (Normalized Innovation Squared)")
print("-" * 40)
if len(accepted) > 0:
    print(f"Mean (accepted):    {accepted['nis'].mean():8.3f}")
    print(f"Median (accepted):  {accepted['nis'].median():8.3f}")
    print(f"Max (accepted):     {accepted['nis'].max():8.3f}")
    print(f"% below gate (7.81): {(accepted['nis'] < 7.81).mean()*100:7.1f}%")
if len(rejected) > 0:
    print(f"Mean (rejected):    {rejected['nis'].mean():8.3f}")
    print(f"Min (rejected):     {rejected['nis'].min():8.3f}")

print(f"\n🎯 ADAPTIVE ALPHA (Soft Update Factor)")
print("-" * 40)
if len(accepted) > 0:
    print(f"Mean:               {accepted['alpha'].mean():8.3f}")
    print(f"Min:                {accepted['alpha'].min():8.3f}")
    print(f"Max:                {accepted['alpha'].max():8.3f}")
    print(f"Std Dev:            {accepted['alpha'].std():8.3f}")

print(f"\n🏔️ TERRAIN SLOPE")
print("-" * 40)
print(f"Mean:               {log['slope'].mean():8.3f} m/m")
print(f"Max:                {log['slope'].max():8.3f} m/m")
print(f"Min:                {log['slope'].min():8.3f} m/m")
print(f"% below floor (0.02): {(log['slope'] < 0.02).mean()*100:6.1f}%")

print(f"\n📏 MEASUREMENT VARIANCE R")
print("-" * 40)
print(f"Mean:               {log['R'].mean():8.3f} m²")
print(f"Min:                {log['R'].min():8.3f} m²")
print(f"Max:                {log['R'].max():8.3f} m²")

print(f"\n🎯 RESIDUALS")
print("-" * 40)
if len(accepted) > 0:
    print(f"Mean (accepted):    {accepted['residual'].mean():8.3f} m")
    print(f"Std (accepted):     {accepted['residual'].std():8.3f} m")
    print(f"Max |r| (accepted): {accepted['residual'].abs().max():8.3f} m")
    print(f"% within ±1.5m:     {(accepted['residual'].abs() < 1.5).mean()*100:7.1f}%")

# Time-based analysis
print(f"\n⏱️ TEMPORAL ANALYSIS")
print("-" * 40)
# Split into quarters
quarters = np.array_split(log, 4)
for i, q in enumerate(quarters):
    t_start = q['t'].min()
    t_end = q['t'].max()
    accept_rate = q['trn'].mean() * 100
    print(f"Quarter {i+1} ({t_start:3.0f}-{t_end:3.0f}s): {accept_rate:5.1f}% accepted")

# Roll angle analysis
print(f"\n✈️ ROLL ANGLE EFFECTS")
print("-" * 40)
print(f"Mean roll:          {np.degrees(log['roll'].mean()):8.2f}°")
print(f"Max |roll|:         {np.degrees(log['roll'].abs().max()):8.2f}°")

# High roll vs low roll acceptance
low_roll = log[log['roll'].abs() < np.radians(5)]
high_roll = log[log['roll'].abs() >= np.radians(5)]
if len(low_roll) > 0:
    print(f"Accept rate (|φ|<5°):  {low_roll['trn'].mean()*100:6.1f}%")
if len(high_roll) > 0:
    print(f"Accept rate (|φ|≥5°):  {high_roll['trn'].mean()*100:6.1f}%")

print("\n" + "=" * 70)