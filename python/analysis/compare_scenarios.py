#!/usr/bin/env python3
"""Compare performance across test scenarios"""

import pandas as pd
import numpy as np

def quick_metrics(output_file, truth_file='data/sim_truth.csv'):
    """Compute key metrics for a run"""
    est = pd.read_csv(output_file)
    truth = pd.read_csv(truth_file)
    
    N = min(len(est), len(truth))
    est = est.iloc[:N]
    truth = truth.iloc[:N]
    
    # Errors
    err_n = est['pn'] - truth['pn']
    err_e = est['pe'] - truth['pe']
    err_d = est['pd'] - truth['pd']
    err_2d = np.sqrt(err_n**2 + err_e**2)
    
    # Steps
    dpn = est['pn'].diff()
    dpe = est['pe'].diff()
    dt = est['t'].diff()
    expected_dpn = est['vn'] * dt
    expected_dpe = est['ve'] * dt
    trn_dpn = dpn - expected_dpn
    trn_dpe = dpe - expected_dpe
    trn_step = np.sqrt(trn_dpn**2 + trn_dpe**2)
    
    return {
        'CEP50': np.percentile(err_2d, 50),
        'CEP90': np.percentile(err_2d, 90),
        'CEP95': np.percentile(err_2d, 95),
        'Final_2D': err_2d.iloc[-1],
        'Max_2D': err_2d.max(),
        'Max_Step': trn_step.max(),
        'Steps_>1m': (trn_step > 1.0).sum(),
        'Steps_>2m': (trn_step > 2.0).sum(),
        'Stable_%': 100 * (err_2d < 5.0).mean()
    }

# Analyze scenarios
scenarios = [
    ('Baseline', 'data/run_output.csv'),
    ('Noisy (+50%)', 'data/run_noisy.csv'),
    ('Aggressive', 'data/run_aggressive.csv')
]

print("=" * 80)
print("SCENARIO COMPARISON - ADAPTIVE TRN PERFORMANCE")
print("=" * 80)
print()
print("Metric            Baseline    Noisy       Aggressive  Target")
print("-" * 80)

metrics_list = []
for name, file in scenarios:
    try:
        m = quick_metrics(file)
        metrics_list.append((name, m))
    except:
        print(f"Could not analyze {name}")
        continue

# Display comparison
metric_names = [
    ('CEP50', '≤2m'),
    ('CEP90', '≤10m'),
    ('CEP95', '≤15m'),
    ('Final_2D', '≤15m'),
    ('Max_2D', '≤25m'),
    ('Max_Step', '≤3m'),
    ('Steps_>1m', '≤3'),
    ('Steps_>2m', '0'),
    ('Stable_%', '≥85%')
]

for metric, target in metric_names:
    row = f"{metric:12s}"
    for name, m in metrics_list:
        val = m[metric]
        if isinstance(val, (int, np.integer)):
            row += f"  {val:10d}"
        else:
            row += f"  {val:10.2f}"
    row += f"  {target}"
    print(row)

print()
print("=" * 80)
print("GRADE ASSESSMENT")
print("-" * 80)

for name, m in metrics_list:
    grade = 'A'
    issues = []
    
    if m['CEP95'] > 15:
        if m['CEP95'] > 20:
            grade = 'B'
            issues.append(f"CEP95={m['CEP95']:.1f}m")
        else:
            grade = 'A-' if grade == 'A' else grade
            issues.append(f"CEP95={m['CEP95']:.1f}m")
    
    if m['Stable_%'] < 85:
        if m['Stable_%'] < 70:
            grade = 'B' if grade in ['A', 'A-'] else grade
            issues.append(f"Stability={m['Stable_%']:.1f}%")
        else:
            grade = 'A-' if grade == 'A' else grade
            issues.append(f"Stability={m['Stable_%']:.1f}%")
    
    if m['Max_Step'] > 3:
        grade = 'B' if grade in ['A', 'A-'] else grade
        issues.append(f"MaxStep={m['Max_Step']:.1f}m")
    
    if m['Steps_>2m'] > 0:
        grade = 'B' if grade in ['A', 'A-'] else grade
        issues.append(f"LargeSteps={m['Steps_>2m']}")
    
    issue_str = ', '.join(issues) if issues else 'All targets met'
    print(f"{name:15s}: Grade {grade:3s} - {issue_str}")

print("=" * 80)