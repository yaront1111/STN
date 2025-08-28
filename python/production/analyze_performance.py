#!/usr/bin/env python3
"""
Analyze STN navigation performance and compute metrics
"""

import pandas as pd
import numpy as np
from pathlib import Path

def analyze_navigation_performance(nav_file, truth_file):
    """
    Analyze navigation performance and compute key metrics
    
    Returns:
        dict with performance metrics
    """
    # Load data
    nav = pd.read_csv(nav_file)
    truth = pd.read_csv(truth_file)
    
    # Align times (in case of different sampling)
    # Interpolate navigation to truth times
    from scipy.interpolate import interp1d
    
    # Create interpolators for navigation data
    f_pn = interp1d(nav['t'], nav['pn'], kind='linear', fill_value='extrapolate')
    f_pe = interp1d(nav['t'], nav['pe'], kind='linear', fill_value='extrapolate')
    f_pd = interp1d(nav['t'], nav['pd'], kind='linear', fill_value='extrapolate')
    
    # Interpolate to truth times
    nav_pn = f_pn(truth['t'])
    nav_pe = f_pe(truth['t'])
    nav_pd = f_pd(truth['t'])
    
    # Compute errors
    err_n = nav_pn - truth['pn']
    err_e = nav_pe - truth['pe']
    err_d = nav_pd - truth['pd']
    
    # Horizontal error
    err_h = np.sqrt(err_n**2 + err_e**2)
    
    # Vertical error
    err_v = np.abs(err_d)
    
    # Key metrics
    cep50 = np.percentile(err_h, 50)  # Median error
    cep95 = np.percentile(err_h, 95)  # 95th percentile
    cep99 = np.percentile(err_h, 99)  # 99th percentile
    
    rms_h = np.sqrt(np.mean(err_h**2))  # RMS horizontal
    rms_v = np.sqrt(np.mean(err_v**2))  # RMS vertical
    
    max_h = np.max(err_h)
    max_v = np.max(err_v)
    
    final_h = err_h[len(err_h)-1] if len(err_h) > 0 else 0
    final_v = err_v[len(err_v)-1] if len(err_v) > 0 else 0
    
    # Stability: percentage within 30m
    stability = 100 * np.sum(err_h < 30) / len(err_h)
    
    # Convergence: time to reach <20m error
    convergence_idx = np.where(err_h < 20)[0]
    if len(convergence_idx) > 0:
        convergence_time = truth['t'].iloc[convergence_idx[0]]
    else:
        convergence_time = np.inf
    
    # Grade determination
    if cep95 <= 15 and stability >= 85:
        grade = 'A'
        grade_desc = 'Excellent - Production ready'
    elif cep95 <= 25 and stability >= 70:
        grade = 'B'
        grade_desc = 'Good - Acceptable performance'
    elif cep95 <= 35 and stability >= 60:
        grade = 'C'
        grade_desc = 'Fair - Needs improvement'
    else:
        grade = 'D'
        grade_desc = 'Poor - Significant issues'
    
    # Create results dictionary
    results = {
        'grade': grade,
        'grade_desc': grade_desc,
        'cep50': cep50,
        'cep95': cep95,
        'cep99': cep99,
        'rms_h': rms_h,
        'rms_v': rms_v,
        'max_h': max_h,
        'max_v': max_v,
        'final_h': final_h,
        'final_v': final_v,
        'stability': stability,
        'convergence_time': convergence_time,
        'mean_h': np.mean(err_h),
        'std_h': np.std(err_h),
        'mean_v': np.mean(err_v),
        'std_v': np.std(err_v),
    }
    
    # Time series data for plotting
    results['time'] = truth['t'].values
    results['err_h'] = err_h
    results['err_v'] = err_v
    results['err_n'] = err_n
    results['err_e'] = err_e
    results['err_d'] = err_d
    
    return results


def print_performance_report(results):
    """Print formatted performance report"""
    
    print("\n" + "="*60)
    print("STN NAVIGATION PERFORMANCE REPORT")
    print("="*60)
    
    # Grade
    print(f"\n📊 GRADE: {results['grade']} - {results['grade_desc']}")
    
    # Key metrics
    print("\n🎯 KEY METRICS:")
    print(f"  CEP50: {results['cep50']:.1f}m (median)")
    print(f"  CEP95: {results['cep95']:.1f}m (95th percentile)")
    print(f"  CEP99: {results['cep99']:.1f}m (99th percentile)")
    
    # RMS errors
    print("\n📐 RMS ERRORS:")
    print(f"  Horizontal: {results['rms_h']:.1f}m")
    print(f"  Vertical: {results['rms_v']:.1f}m")
    
    # Maximum errors
    print("\n⚠️ MAXIMUM ERRORS:")
    print(f"  Horizontal: {results['max_h']:.1f}m")
    print(f"  Vertical: {results['max_v']:.1f}m")
    
    # Final errors
    print("\n🏁 FINAL ERRORS:")
    print(f"  Horizontal: {results['final_h']:.1f}m")
    print(f"  Vertical: {results['final_v']:.1f}m")
    
    # Stability and convergence
    print("\n📈 STABILITY & CONVERGENCE:")
    print(f"  Stability: {results['stability']:.1f}% (within 30m)")
    if results['convergence_time'] < np.inf:
        print(f"  Convergence: {results['convergence_time']:.1f}s to reach <20m")
    else:
        print(f"  Convergence: Never reached <20m")
    
    # Statistics
    print("\n📊 STATISTICS:")
    print(f"  Horizontal: {results['mean_h']:.1f} ± {results['std_h']:.1f}m")
    print(f"  Vertical: {results['mean_v']:.1f} ± {results['std_v']:.1f}m")
    
    # Grade requirements
    print("\n🎯 GRADE A REQUIREMENTS:")
    print(f"  CEP95 ≤15m: {'✅' if results['cep95'] <= 15 else '❌'} (current: {results['cep95']:.1f}m)")
    print(f"  Stability ≥85%: {'✅' if results['stability'] >= 85 else '❌'} (current: {results['stability']:.1f}%)")
    
    if results['grade'] != 'A':
        print("\n💡 TO ACHIEVE GRADE A:")
        if results['cep95'] > 15:
            improvement = results['cep95'] - 15
            print(f"  - Reduce CEP95 by {improvement:.1f}m")
        if results['stability'] < 85:
            improvement = 85 - results['stability']
            print(f"  - Improve stability by {improvement:.1f}%")
    
    print("\n" + "="*60)


if __name__ == "__main__":
    # Analyze the latest navigation run
    nav_file = "data/flight/nav_output.csv"
    truth_file = "data/flight/truth.csv"
    
    if not Path(nav_file).exists():
        print(f"Navigation file not found: {nav_file}")
        print("Please run navigation first:")
        print("  ./build/stn_navigator data/flight/imu.csv data/flight/nav_output.csv data/flight/radalt.csv")
    elif not Path(truth_file).exists():
        print(f"Truth file not found: {truth_file}")
    else:
        results = analyze_navigation_performance(nav_file, truth_file)
        print_performance_report(results)
        
        # Save results
        results_df = pd.DataFrame({
            'metric': list(results.keys()),
            'value': list(results.values())
        })
        results_file = "data/flight/performance_metrics.csv"
        results_df.to_csv(results_file, index=False)
        print(f"\nMetrics saved to: {results_file}")