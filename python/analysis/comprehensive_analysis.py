#!/usr/bin/env python3
"""
Comprehensive STN System Analysis for Real-World Optimization
Analyzes performance, smoothness, and robustness metrics
"""

import pandas as pd
import numpy as np
from pathlib import Path
import json

class STNAnalyzer:
    def __init__(self, est_file='data/run_output.csv', 
                 truth_file='data/sim_truth.csv',
                 radalt_file='data/radalt.csv'):
        """Initialize analyzer with data files"""
        self.est = pd.read_csv(est_file)
        self.truth = pd.read_csv(truth_file)
        self.radalt = pd.read_csv(radalt_file) if Path(radalt_file).exists() else None
        
        # Align data
        N = min(len(self.est), len(self.truth))
        self.est = self.est.iloc[:N].reset_index(drop=True)
        self.truth = self.truth.iloc[:N].reset_index(drop=True)
        
        # Compute derived quantities
        self._compute_errors()
        self._detect_steps()
        
    def _compute_errors(self):
        """Compute position and velocity errors"""
        self.errors = pd.DataFrame()
        self.errors['t'] = self.est['t']
        self.errors['err_n'] = self.est['pn'] - self.truth['pn']
        self.errors['err_e'] = self.est['pe'] - self.truth['pe']
        self.errors['err_d'] = self.est['pd'] - self.truth['pd']
        self.errors['err_2d'] = np.sqrt(self.errors['err_n']**2 + self.errors['err_e']**2)
        self.errors['err_3d'] = np.sqrt(self.errors['err_n']**2 + 
                                       self.errors['err_e']**2 + 
                                       self.errors['err_d']**2)
        
        # Velocity errors
        self.errors['err_vn'] = self.est['vn'] - self.truth['vn']
        self.errors['err_ve'] = self.est['ve'] - self.truth['ve']
        self.errors['err_vd'] = self.est['vd'] - self.truth['vd']
        
    def _detect_steps(self):
        """Detect position steps and TRN updates"""
        self.est['dpn'] = self.est['pn'].diff()
        self.est['dpe'] = self.est['pe'].diff()
        self.est['dpd'] = self.est['pd'].diff()
        self.est['dt'] = self.est['t'].diff()
        
        # Expected step from velocity
        self.est['expected_dpn'] = self.est['vn'] * self.est['dt']
        self.est['expected_dpe'] = self.est['ve'] * self.est['dt']
        
        # Step sizes
        self.est['step_2d'] = np.sqrt(self.est['dpn']**2 + self.est['dpe']**2)
        self.est['step_3d'] = np.sqrt(self.est['dpn']**2 + self.est['dpe']**2 + self.est['dpd']**2)
        
        # Detect TRN corrections (steps that deviate from expected)
        self.est['trn_dpn'] = self.est['dpn'] - self.est['expected_dpn']
        self.est['trn_dpe'] = self.est['dpe'] - self.est['expected_dpe']
        self.est['trn_step'] = np.sqrt(self.est['trn_dpn']**2 + self.est['trn_dpe']**2)
        self.est['is_trn'] = self.est['trn_step'] > 0.1  # TRN if correction > 10cm
        
    def accuracy_metrics(self):
        """Compute accuracy metrics for real-world use"""
        metrics = {}
        
        # Position accuracy
        metrics['rmse_2d'] = np.sqrt(np.mean(self.errors['err_2d']**2))
        metrics['rmse_3d'] = np.sqrt(np.mean(self.errors['err_3d']**2))
        metrics['cep50'] = np.percentile(self.errors['err_2d'], 50)
        metrics['cep90'] = np.percentile(self.errors['err_2d'], 90)
        metrics['cep95'] = np.percentile(self.errors['err_2d'], 95)
        metrics['max_2d_error'] = self.errors['err_2d'].max()
        metrics['final_2d_error'] = self.errors['err_2d'].iloc[-1]
        
        # Vertical accuracy
        metrics['rmse_vertical'] = np.sqrt(np.mean(self.errors['err_d']**2))
        metrics['max_vertical_error'] = np.abs(self.errors['err_d']).max()
        
        # Velocity accuracy
        metrics['rmse_vn'] = np.sqrt(np.mean(self.errors['err_vn']**2))
        metrics['rmse_ve'] = np.sqrt(np.mean(self.errors['err_ve']**2))
        metrics['rmse_vd'] = np.sqrt(np.mean(self.errors['err_vd']**2))
        
        return metrics
    
    def smoothness_metrics(self):
        """Compute smoothness metrics critical for real-world use"""
        metrics = {}
        
        # Step statistics
        trn_steps = self.est[self.est['is_trn']]
        metrics['num_trn_updates'] = len(trn_steps)
        
        if len(trn_steps) > 0:
            metrics['mean_trn_step'] = trn_steps['trn_step'].mean()
            metrics['max_trn_step'] = trn_steps['trn_step'].max()
            metrics['median_trn_step'] = trn_steps['trn_step'].median()
            
            # Step size distribution
            for threshold in [0.5, 1.0, 2.0, 5.0, 10.0]:
                metrics[f'steps_over_{threshold}m'] = (trn_steps['trn_step'] > threshold).sum()
        
        # Velocity smoothness (acceleration)
        self.est['an'] = self.est['vn'].diff() / self.est['dt']
        self.est['ae'] = self.est['ve'].diff() / self.est['dt']
        self.est['ad'] = self.est['vd'].diff() / self.est['dt']
        self.est['a_mag'] = np.sqrt(self.est['an']**2 + self.est['ae']**2 + self.est['ad']**2)
        
        metrics['mean_acceleration'] = self.est['a_mag'].mean()
        metrics['max_acceleration'] = self.est['a_mag'].max()
        metrics['acceleration_std'] = self.est['a_mag'].std()
        
        # Jerk (rate of acceleration change) - important for passenger comfort
        self.est['jerk'] = self.est['a_mag'].diff() / self.est['dt']
        metrics['mean_jerk'] = np.abs(self.est['jerk']).mean()
        metrics['max_jerk'] = np.abs(self.est['jerk']).max()
        
        return metrics
    
    def robustness_metrics(self):
        """Compute robustness metrics for real-world reliability"""
        metrics = {}
        
        # Error growth rate
        window = 100  # 1 second window
        error_growth = []
        for i in range(0, len(self.errors) - window, window):
            e1 = self.errors['err_2d'].iloc[i]
            e2 = self.errors['err_2d'].iloc[i + window]
            dt = self.errors['t'].iloc[i + window] - self.errors['t'].iloc[i]
            if dt > 0:
                growth_rate = (e2 - e1) / dt
                error_growth.append(growth_rate)
        
        if error_growth:
            metrics['mean_error_growth_rate'] = np.mean(error_growth)
            metrics['max_error_growth_rate'] = np.max(error_growth)
        
        # Consistency: how often TRN improves vs degrades
        improvements = 0
        degradations = 0
        
        trn_indices = self.est[self.est['is_trn']].index
        for idx in trn_indices:
            if idx > 0 and idx < len(self.errors) - 1:
                err_before = self.errors['err_2d'].iloc[idx - 1]
                err_after = self.errors['err_2d'].iloc[idx + 1]
                if err_after < err_before:
                    improvements += 1
                else:
                    degradations += 1
        
        metrics['trn_improvements'] = improvements
        metrics['trn_degradations'] = degradations
        if improvements + degradations > 0:
            metrics['trn_success_rate'] = improvements / (improvements + degradations)
        
        # Stability zones (periods of low error)
        stable_threshold = 5.0  # Consider stable if error < 5m
        stable_periods = []
        in_stable = False
        start_time = 0
        
        for i, row in self.errors.iterrows():
            if row['err_2d'] < stable_threshold:
                if not in_stable:
                    in_stable = True
                    start_time = row['t']
            else:
                if in_stable:
                    duration = row['t'] - start_time
                    if duration > 1.0:  # Only count periods > 1s
                        stable_periods.append(duration)
                    in_stable = False
        
        if stable_periods:
            metrics['num_stable_periods'] = len(stable_periods)
            metrics['mean_stable_duration'] = np.mean(stable_periods)
            metrics['total_stable_time'] = sum(stable_periods)
            metrics['stability_percentage'] = 100 * sum(stable_periods) / self.errors['t'].iloc[-1]
        
        return metrics
    
    def real_world_score(self):
        """Compute overall score for real-world usability"""
        acc = self.accuracy_metrics()
        smooth = self.smoothness_metrics()
        robust = self.robustness_metrics()
        
        # Weighted scoring (adjust weights based on application)
        score_components = {
            'accuracy': 0,
            'smoothness': 0,
            'robustness': 0
        }
        
        # Accuracy score (40% weight)
        # Penalize based on CEP95 and final error
        acc_penalty = 0
        if acc['cep95'] < 10:
            acc_penalty = 0
        elif acc['cep95'] < 30:
            acc_penalty = 0.2
        elif acc['cep95'] < 50:
            acc_penalty = 0.4
        else:
            acc_penalty = 0.6
            
        if acc['final_2d_error'] > 50:
            acc_penalty += 0.2
            
        score_components['accuracy'] = max(0, 1.0 - acc_penalty) * 40
        
        # Smoothness score (30% weight)
        # Penalize large steps and high jerk
        smooth_penalty = 0
        if smooth.get('max_trn_step', 0) > 10:
            smooth_penalty += 0.3
        elif smooth.get('max_trn_step', 0) > 5:
            smooth_penalty += 0.1
            
        if smooth.get('steps_over_2.0m', 0) > 10:
            smooth_penalty += 0.2
            
        if smooth.get('max_jerk', 0) > 100:
            smooth_penalty += 0.2
            
        score_components['smoothness'] = max(0, 1.0 - smooth_penalty) * 30
        
        # Robustness score (30% weight)
        # Reward high success rate and stability
        robust_score = 0
        if robust.get('trn_success_rate', 0) > 0.6:
            robust_score += 0.4
        elif robust.get('trn_success_rate', 0) > 0.5:
            robust_score += 0.2
            
        if robust.get('stability_percentage', 0) > 50:
            robust_score += 0.3
        elif robust.get('stability_percentage', 0) > 30:
            robust_score += 0.2
            
        if robust.get('mean_error_growth_rate', float('inf')) < 0.1:
            robust_score += 0.3
            
        score_components['robustness'] = min(1.0, robust_score) * 30
        
        total_score = sum(score_components.values())
        
        return {
            'total_score': total_score,
            'components': score_components,
            'grade': self._get_grade(total_score)
        }
    
    def _get_grade(self, score):
        """Convert score to letter grade"""
        if score >= 90:
            return 'A'
        elif score >= 80:
            return 'B'
        elif score >= 70:
            return 'C'
        elif score >= 60:
            return 'D'
        else:
            return 'F'
    
    def print_report(self):
        """Print comprehensive analysis report"""
        print("=" * 70)
        print("STN SYSTEM COMPREHENSIVE ANALYSIS REPORT")
        print("=" * 70)
        
        acc = self.accuracy_metrics()
        print("\n📊 ACCURACY METRICS")
        print("-" * 40)
        print(f"2D RMSE:           {acc['rmse_2d']:8.2f} m")
        print(f"CEP50:             {acc['cep50']:8.2f} m")
        print(f"CEP90:             {acc['cep90']:8.2f} m")
        print(f"CEP95:             {acc['cep95']:8.2f} m")
        print(f"Final 2D Error:    {acc['final_2d_error']:8.2f} m")
        print(f"Max 2D Error:      {acc['max_2d_error']:8.2f} m")
        print(f"Vertical RMSE:     {acc['rmse_vertical']:8.3f} m")
        
        smooth = self.smoothness_metrics()
        print("\n🌊 SMOOTHNESS METRICS")
        print("-" * 40)
        print(f"TRN Updates:       {smooth['num_trn_updates']:8d}")
        if smooth['num_trn_updates'] > 0:
            print(f"Mean TRN Step:     {smooth['mean_trn_step']:8.2f} m")
            print(f"Max TRN Step:      {smooth['max_trn_step']:8.2f} m")
            print(f"Steps > 1m:        {smooth.get('steps_over_1.0m', 0):8d}")
            print(f"Steps > 2m:        {smooth.get('steps_over_2.0m', 0):8d}")
            print(f"Steps > 5m:        {smooth.get('steps_over_5.0m', 0):8d}")
        print(f"Max Acceleration:  {smooth['max_acceleration']:8.2f} m/s²")
        print(f"Max Jerk:          {smooth['max_jerk']:8.2f} m/s³")
        
        robust = self.robustness_metrics()
        print("\n🛡️ ROBUSTNESS METRICS")
        print("-" * 40)
        if 'trn_success_rate' in robust:
            print(f"TRN Success Rate:  {robust['trn_success_rate']*100:8.1f} %")
            print(f"TRN Improvements:  {robust['trn_improvements']:8d}")
            print(f"TRN Degradations:  {robust['trn_degradations']:8d}")
        if 'stability_percentage' in robust:
            print(f"Stability Time:    {robust['stability_percentage']:8.1f} %")
            print(f"Stable Periods:    {robust['num_stable_periods']:8d}")
        if 'mean_error_growth_rate' in robust:
            print(f"Error Growth Rate: {robust['mean_error_growth_rate']:8.3f} m/s")
        
        score = self.real_world_score()
        print("\n🎯 REAL-WORLD USABILITY SCORE")
        print("-" * 40)
        print(f"Total Score:       {score['total_score']:8.1f} / 100")
        print(f"Grade:             {score['grade']:>8s}")
        print("\nComponent Scores:")
        for comp, val in score['components'].items():
            print(f"  {comp.capitalize():12s}   {val:5.1f} / {40 if comp=='accuracy' else 30}")
        
        # Recommendations
        print("\n💡 RECOMMENDATIONS")
        print("-" * 40)
        recommendations = []
        
        if acc['cep95'] > 30:
            recommendations.append("• Accuracy needs improvement - consider increasing TRN rate or improving terrain matching")
        
        if smooth.get('max_trn_step', 0) > 5:
            recommendations.append("• Large step artifacts detected - reduce max step limits or improve filtering")
        
        if robust.get('trn_success_rate', 0) < 0.6:
            recommendations.append("• Low TRN success rate - improve gating or terrain gradient calculation")
        
        if robust.get('stability_percentage', 0) < 30:
            recommendations.append("• Low stability - tune process noise and measurement covariance")
        
        if not recommendations:
            recommendations.append("✓ System performing well for real-world use")
        
        for rec in recommendations:
            print(rec)
        
        print("\n" + "=" * 70)
        
    def save_metrics(self, filename='data/metrics.json'):
        """Save all metrics to JSON file"""
        def convert_to_serializable(obj):
            """Convert numpy types to Python native types"""
            if isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [convert_to_serializable(item) for item in obj]
            elif isinstance(obj, (np.integer, np.int64)):
                return int(obj)
            elif isinstance(obj, (np.float64, np.float32)):
                return float(obj)
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            else:
                return obj
        
        metrics = {
            'accuracy': self.accuracy_metrics(),
            'smoothness': self.smoothness_metrics(),
            'robustness': self.robustness_metrics(),
            'score': self.real_world_score()
        }
        
        metrics_serializable = convert_to_serializable(metrics)
        
        with open(filename, 'w') as f:
            json.dump(metrics_serializable, f, indent=2)
        print(f"Metrics saved to {filename}")


def main():
    """Run comprehensive analysis"""
    analyzer = STNAnalyzer()
    analyzer.print_report()
    analyzer.save_metrics()


if __name__ == "__main__":
    main()