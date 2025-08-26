#!/usr/bin/env python3
"""
Monte Carlo Analysis for STN Navigation System
Statistical validation through multiple simulation runs
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import multiprocessing as mp
from dataclasses import dataclass
import json
import subprocess
import tempfile
import sys
sys.path.append('../sim')
from trajectory_generator import TrajectoryGenerator, IMUConfig

@dataclass
class MonteCarloConfig:
    """Configuration for Monte Carlo simulation"""
    num_runs: int = 100
    trajectory_type: str = 'figure8'  # straight, turn, climb, figure8
    duration: float = 60.0
    imu_quality: str = 'tactical'
    
    # Parameter variations
    vary_imu_noise: bool = True
    vary_initial_error: bool = True
    vary_terrain: bool = False
    
    # Initial error ranges
    pos_error_range: Tuple[float, float] = (0, 50)  # meters
    vel_error_range: Tuple[float, float] = (0, 5)   # m/s
    att_error_range: Tuple[float, float] = (0, 0.1) # radians
    
    # Output settings
    output_dir: str = 'monte_carlo_results'
    save_all_runs: bool = False
    parallel_runs: int = 4


class MonteCarloRunner:
    """Runs Monte Carlo simulations of STN navigation"""
    
    def __init__(self, config: MonteCarloConfig, stn_executable: str = 'build/stn_demo'):
        self.config = config
        self.stn_executable = Path(stn_executable)
        if not self.stn_executable.exists():
            raise FileNotFoundError(f"STN executable not found: {stn_executable}")
        
        self.output_dir = Path(config.output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.generator = TrajectoryGenerator()
        
    def generate_scenario(self, run_id: int) -> Dict:
        """Generate a single scenario with random variations"""
        
        # Generate trajectory
        if self.config.trajectory_type == 'straight':
            imu_perfect, truth = self.generator.generate_straight_level(
                self.config.duration, 50, 1000)
        elif self.config.trajectory_type == 'turn':
            imu_perfect, truth = self.generator.generate_coordinated_turn(
                self.config.duration, 50, 1000, 3)
        elif self.config.trajectory_type == 'climb':
            imu_perfect, truth = self.generator.generate_climb_descend(
                self.config.duration, 50, 1000, 1500)
        else:  # figure8
            imu_perfect, truth = self.generator.generate_figure_eight(
                self.config.duration, 50, 1000)
        
        # Add IMU noise
        if self.config.vary_imu_noise:
            # Random noise scale factor
            noise_scale = np.random.uniform(0.5, 2.0)
            imu_config = getattr(IMUConfig, self.config.imu_quality)()
            imu_config.gyro_bias_stability *= noise_scale
            imu_config.accel_bias_stability *= noise_scale
            imu_noisy = self.generator.add_imu_errors(imu_perfect, imu_config)
        else:
            imu_noisy = imu_perfect
        
        # Generate initial errors
        if self.config.vary_initial_error:
            initial_errors = {
                'pos': np.random.uniform(*self.config.pos_error_range, 3),
                'vel': np.random.uniform(*self.config.vel_error_range, 3),
                'att': np.random.uniform(*self.config.att_error_range, 3)
            }
        else:
            initial_errors = {
                'pos': np.zeros(3),
                'vel': np.zeros(3),
                'att': np.zeros(3)
            }
        
        # Apply initial errors to IMU data (simplified)
        # In reality, this would be done in the navigation system initialization
        
        return {
            'run_id': run_id,
            'imu': imu_noisy,
            'truth': truth,
            'initial_errors': initial_errors,
            'noise_scale': noise_scale if self.config.vary_imu_noise else 1.0
        }
    
    def run_single_simulation(self, scenario: Dict) -> Dict:
        """Run a single STN simulation"""
        
        run_id = scenario['run_id']
        
        # Save IMU data to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            scenario['imu'].to_csv(f, index=False)
            imu_file = f.name
        
        # Save truth data
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            scenario['truth'].to_csv(f, index=False)
            truth_file = f.name
        
        # Output file
        output_file = self.output_dir / f'run_{run_id:04d}.csv'
        
        # Run STN navigation
        try:
            result = subprocess.run(
                [str(self.stn_executable), imu_file, str(output_file)],
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode != 0:
                print(f"Run {run_id} failed: {result.stderr}")
                return None
            
            # Load navigation output
            nav_df = pd.read_csv(output_file)
            
            # Calculate errors
            errors = self.calculate_errors(nav_df, scenario['truth'])
            
            # Store results
            results = {
                'run_id': run_id,
                'success': True,
                'errors': errors,
                'initial_errors': scenario['initial_errors'],
                'noise_scale': scenario['noise_scale']
            }
            
            # Clean up if not saving all runs
            if not self.config.save_all_runs:
                output_file.unlink()
            
        except subprocess.TimeoutExpired:
            print(f"Run {run_id} timed out")
            results = {'run_id': run_id, 'success': False}
        
        finally:
            # Clean up temp files
            Path(imu_file).unlink()
            Path(truth_file).unlink()
        
        return results
    
    def calculate_errors(self, nav_df: pd.DataFrame, truth_df: pd.DataFrame) -> Dict:
        """Calculate navigation errors"""
        
        # Align timestamps (simple nearest neighbor)
        merged = pd.merge_asof(
            nav_df.sort_values('t'),
            truth_df.sort_values('t'),
            on='t',
            direction='nearest',
            suffixes=('', '_truth')
        )
        
        # Position errors
        pos_error = np.sqrt(
            (merged['pn'] - merged['pn_truth'])**2 +
            (merged['pe'] - merged['pe_truth'])**2 +
            (merged['pd'] - merged['pd_truth'])**2
        )
        
        # Velocity errors
        vel_error = np.sqrt(
            (merged['vn'] - merged['vn_truth'])**2 +
            (merged['ve'] - merged['ve_truth'])**2 +
            (merged['vd'] - merged['vd_truth'])**2
        )
        
        # Horizontal error
        horz_error = np.sqrt(
            (merged['pn'] - merged['pn_truth'])**2 +
            (merged['pe'] - merged['pe_truth'])**2
        )
        
        return {
            'pos_mean': pos_error.mean(),
            'pos_max': pos_error.max(),
            'pos_final': pos_error.iloc[-1],
            'pos_rms': np.sqrt((pos_error**2).mean()),
            'vel_mean': vel_error.mean(),
            'vel_max': vel_error.max(),
            'horz_mean': horz_error.mean(),
            'horz_final': horz_error.iloc[-1],
            'cep50': np.percentile(horz_error, 50),
            'cep95': np.percentile(horz_error, 95)
        }
    
    def run_parallel(self, scenarios: List[Dict]) -> List[Dict]:
        """Run simulations in parallel"""
        
        with mp.Pool(processes=self.config.parallel_runs) as pool:
            results = pool.map(self.run_single_simulation, scenarios)
        
        return [r for r in results if r is not None]
    
    def run(self) -> pd.DataFrame:
        """Run complete Monte Carlo analysis"""
        
        print(f"Starting Monte Carlo analysis with {self.config.num_runs} runs")
        print(f"Trajectory: {self.config.trajectory_type}, Duration: {self.config.duration}s")
        print(f"IMU Quality: {self.config.imu_quality}")
        print(f"Output directory: {self.output_dir}")
        
        # Generate all scenarios
        scenarios = [self.generate_scenario(i) for i in range(self.config.num_runs)]
        
        # Run simulations
        if self.config.parallel_runs > 1:
            print(f"Running simulations in parallel ({self.config.parallel_runs} processes)...")
            results = self.run_parallel(scenarios)
        else:
            print("Running simulations sequentially...")
            results = [self.run_single_simulation(s) for s in scenarios]
        
        # Filter successful runs
        successful = [r for r in results if r and r['success']]
        print(f"Completed {len(successful)}/{self.config.num_runs} successful runs")
        
        # Convert to DataFrame
        df = pd.DataFrame(successful)
        
        # Expand error dictionaries
        error_df = pd.DataFrame(df['errors'].tolist())
        df = pd.concat([df.drop('errors', axis=1), error_df], axis=1)
        
        # Save results
        results_file = self.output_dir / 'monte_carlo_results.csv'
        df.to_csv(results_file, index=False)
        print(f"Results saved to {results_file}")
        
        return df
    
    def analyze_results(self, df: pd.DataFrame):
        """Generate statistical analysis and plots"""
        
        print("\n=== Monte Carlo Analysis Results ===\n")
        
        # Basic statistics
        print("Position Error Statistics (meters):")
        print(f"  Mean ± Std:     {df['pos_mean'].mean():.2f} ± {df['pos_mean'].std():.2f}")
        print(f"  Max (mean):     {df['pos_max'].mean():.2f}")
        print(f"  Final (mean):   {df['pos_final'].mean():.2f}")
        print(f"  RMS (mean):     {df['pos_rms'].mean():.2f}")
        
        print("\nHorizontal Error Statistics (meters):")
        print(f"  CEP50 (mean):   {df['cep50'].mean():.2f}")
        print(f"  CEP95 (mean):   {df['cep95'].mean():.2f}")
        print(f"  Final (mean):   {df['horz_final'].mean():.2f}")
        
        print("\nVelocity Error Statistics (m/s):")
        print(f"  Mean ± Std:     {df['vel_mean'].mean():.3f} ± {df['vel_mean'].std():.3f}")
        
        # Success rate
        success_rate = len(df) / self.config.num_runs * 100
        print(f"\nSuccess Rate: {success_rate:.1f}%")
        
        # Performance grades
        cep95_mean = df['cep95'].mean()
        if cep95_mean < 10:
            grade = "A (Excellent)"
        elif cep95_mean < 20:
            grade = "B (Good)"
        elif cep95_mean < 50:
            grade = "C (Acceptable)"
        else:
            grade = "D (Poor)"
        print(f"\nPerformance Grade: {grade}")
        
        # Generate plots
        self.generate_plots(df)
    
    def generate_plots(self, df: pd.DataFrame):
        """Generate analysis plots"""
        
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Error distributions
        ax = axes[0, 0]
        ax.hist(df['pos_final'], bins=30, edgecolor='black', alpha=0.7)
        ax.axvline(df['pos_final'].mean(), color='red', linestyle='--', label='Mean')
        ax.set_xlabel('Final Position Error (m)')
        ax.set_ylabel('Count')
        ax.set_title('Final Position Error Distribution')
        ax.legend()
        
        # CEP analysis
        ax = axes[0, 1]
        ax.hist(df['cep50'], bins=30, alpha=0.5, label='CEP50', edgecolor='blue')
        ax.hist(df['cep95'], bins=30, alpha=0.5, label='CEP95', edgecolor='red')
        ax.set_xlabel('Circular Error Probable (m)')
        ax.set_ylabel('Count')
        ax.set_title('CEP Distribution')
        ax.legend()
        
        # Error vs noise scale (if varied)
        if self.config.vary_imu_noise:
            ax = axes[0, 2]
            ax.scatter(df['noise_scale'], df['pos_final'], alpha=0.5)
            ax.set_xlabel('IMU Noise Scale Factor')
            ax.set_ylabel('Final Position Error (m)')
            ax.set_title('Error vs IMU Noise')
            
            # Fit trend line
            z = np.polyfit(df['noise_scale'], df['pos_final'], 1)
            p = np.poly1d(z)
            x_line = np.linspace(df['noise_scale'].min(), df['noise_scale'].max(), 100)
            ax.plot(x_line, p(x_line), "r-", alpha=0.8, label=f'Slope: {z[0]:.1f}')
            ax.legend()
        
        # Success criteria
        ax = axes[1, 0]
        criteria = {
            'CEP50 < 5m': (df['cep50'] < 5).sum() / len(df) * 100,
            'CEP95 < 15m': (df['cep95'] < 15).sum() / len(df) * 100,
            'Final < 20m': (df['pos_final'] < 20).sum() / len(df) * 100,
            'Max < 50m': (df['pos_max'] < 50).sum() / len(df) * 100
        }
        ax.bar(range(len(criteria)), list(criteria.values()))
        ax.set_xticks(range(len(criteria)))
        ax.set_xticklabels(list(criteria.keys()), rotation=45)
        ax.set_ylabel('Success Rate (%)')
        ax.set_title('Performance Criteria Success Rates')
        ax.axhline(y=95, color='g', linestyle='--', alpha=0.5, label='95% target')
        ax.axhline(y=80, color='y', linestyle='--', alpha=0.5, label='80% target')
        ax.legend()
        
        # Velocity errors
        ax = axes[1, 1]
        ax.hist(df['vel_mean'], bins=30, edgecolor='black', alpha=0.7, color='green')
        ax.set_xlabel('Mean Velocity Error (m/s)')
        ax.set_ylabel('Count')
        ax.set_title('Velocity Error Distribution')
        
        # Correlation matrix
        ax = axes[1, 2]
        corr_cols = ['pos_mean', 'vel_mean', 'cep50', 'cep95']
        if self.config.vary_imu_noise:
            corr_cols.append('noise_scale')
        corr_matrix = df[corr_cols].corr()
        sns.heatmap(corr_matrix, annot=True, fmt='.2f', ax=ax, cmap='coolwarm')
        ax.set_title('Error Correlation Matrix')
        
        plt.tight_layout()
        plot_file = self.output_dir / 'monte_carlo_analysis.png'
        plt.savefig(plot_file, dpi=150)
        print(f"\nPlots saved to {plot_file}")
        plt.show()


def main():
    """Run Monte Carlo analysis with command-line arguments"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Monte Carlo analysis for STN')
    parser.add_argument('--runs', type=int, default=100, help='Number of Monte Carlo runs')
    parser.add_argument('--trajectory', choices=['straight', 'turn', 'climb', 'figure8'],
                       default='figure8', help='Trajectory type')
    parser.add_argument('--duration', type=float, default=60, help='Simulation duration (s)')
    parser.add_argument('--imu', choices=['consumer', 'tactical', 'navigation'],
                       default='tactical', help='IMU quality level')
    parser.add_argument('--parallel', type=int, default=4, help='Number of parallel processes')
    parser.add_argument('--output', default='monte_carlo_results', help='Output directory')
    args = parser.parse_args()
    
    # Configure Monte Carlo
    config = MonteCarloConfig(
        num_runs=args.runs,
        trajectory_type=args.trajectory,
        duration=args.duration,
        imu_quality=args.imu,
        parallel_runs=args.parallel,
        output_dir=args.output
    )
    
    # Run analysis
    runner = MonteCarloRunner(config)
    results_df = runner.run()
    
    # Analyze and plot
    runner.analyze_results(results_df)


if __name__ == '__main__':
    main()