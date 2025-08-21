#!/usr/bin/env python3
"""Check if we've achieved Grade A performance"""

import pandas as pd
import numpy as np

def check_grade_a(output_file='data/run_output.csv', truth_file='data/sim_truth.csv'):
    """Check Grade A criteria"""
    est = pd.read_csv(output_file)
    truth = pd.read_csv(truth_file)
    
    N = min(len(est), len(truth))
    est = est.iloc[:N]
    truth = truth.iloc[:N]
    
    # Compute errors
    err_n = est['pn'] - truth['pn']
    err_e = est['pe'] - truth['pe']
    err_2d = np.sqrt(err_n**2 + err_e**2)
    
    # Compute steps
    dpn = est['pn'].diff()
    dpe = est['pe'].diff()
    dt = est['t'].diff()
    expected_dpn = est['vn'] * dt
    expected_dpe = est['ve'] * dt
    trn_step = np.sqrt((dpn - expected_dpn)**2 + (dpe - expected_dpe)**2)
    
    # Metrics
    cep50 = np.percentile(err_2d, 50)
    cep90 = np.percentile(err_2d, 90)
    cep95 = np.percentile(err_2d, 95)
    final_2d = err_2d.iloc[-1]
    max_step = trn_step.max()
    steps_over_3m = (trn_step > 3.0).sum()
    stable_pct = 100 * (err_2d < 5.0).mean()
    
    print("=" * 70)
    print("GRADE A ASSESSMENT")
    print("=" * 70)
    print()
    print("Metric                Current      Target       Status")
    print("-" * 70)
    
    grade = 'A'
    
    # Check each criterion
    def check(name, value, target, op='<='):
        nonlocal grade
        if op == '<=':
            passed = value <= target
            symbol = '✓' if passed else '✗'
        else:  # '>='
            passed = value >= target
            symbol = '✓' if passed else '✗'
        
        if not passed:
            if name in ['CEP95', 'Stable Time']:
                grade = 'B' if grade == 'A' else grade
            else:
                grade = 'A-' if grade == 'A' else grade
        
        if isinstance(value, int):
            print(f"{name:20s}  {value:8d}     {op}{target:6}       {symbol}")
        else:
            print(f"{name:20s}  {value:8.2f}     {op}{target:6.1f}       {symbol}")
        
        return passed
    
    # Grade A criteria
    c1 = check('CEP50 (m)', cep50, 2.0)
    c2 = check('CEP90 (m)', cep90, 10.0)
    c3 = check('CEP95 (m)', cep95, 15.0)  # Key criterion
    c4 = check('Final 2D (m)', final_2d, 15.0)
    c5 = check('Max Step (m)', max_step, 3.0)
    c6 = check('Steps >3m', steps_over_3m, 0)
    c7 = check('Stable Time (%)', stable_pct, 85.0, '>=')  # Key criterion
    
    print()
    print("-" * 70)
    
    if c3 and c7:  # Both key criteria met
        print(f"GRADE: A")
        print("Congratulations! Grade A performance achieved!")
    elif c3 or (cep95 < 17 and stable_pct > 80):
        print(f"GRADE: A-")
        print("Very close to Grade A. Fine-tune alpha or add baro.")
    elif cep95 < 20:
        print(f"GRADE: B+")
        print("Good performance. Increase TRN updates or alpha.")
    else:
        print(f"GRADE: B")
        print("Solid performance but needs improvement.")
    
    print()
    print("Current Performance:")
    print(f"  • CEP95: {cep95:.1f}m (target ≤15m)")
    print(f"  • Stability: {stable_pct:.1f}% (target ≥85%)")
    print(f"  • Final error: {final_2d:.1f}m")
    print(f"  • Smoothness: max step {max_step:.2f}m")
    
    print("=" * 70)

if __name__ == "__main__":
    check_grade_a()