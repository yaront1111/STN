#!/usr/bin/env python3
"""
STN Navigation System - Comprehensive Test Suite
Runs all tests and validates system performance
"""

import subprocess
import sys
import time
import json
from pathlib import Path
from datetime import datetime
import pandas as pd
import numpy as np

class TestRunner:
    def __init__(self):
        self.base_path = Path(__file__).parent
        self.results = {
            'timestamp': datetime.now().isoformat(),
            'tests': {},
            'summary': {}
        }
        
    def print_header(self, title):
        """Print formatted section header"""
        print("\n" + "="*60)
        print(f"  {title}")
        print("="*60)
        
    def run_command(self, cmd, description, cwd=None):
        """Run a command and capture output"""
        print(f"\n▶ {description}...")
        
        if cwd is None:
            cwd = self.base_path
            
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                cwd=str(cwd),
                timeout=60
            )
            
            if result.returncode == 0:
                print(f"✅ {description} - SUCCESS")
                return True, result.stdout
            else:
                print(f"❌ {description} - FAILED")
                print(f"   Error: {result.stderr[:200]}")
                return False, result.stderr
                
        except subprocess.TimeoutExpired:
            print(f"⏱️ {description} - TIMEOUT")
            return False, "Command timed out"
        except Exception as e:
            print(f"❌ {description} - ERROR: {str(e)}")
            return False, str(e)
            
    def test_build_system(self):
        """Test that the system builds correctly"""
        self.print_header("BUILD SYSTEM TEST")
        
        build_path = self.base_path / 'build'
        build_path.mkdir(exist_ok=True)
        
        # Clean build
        success, _ = self.run_command(
            ['rm', '-rf', '*'],
            "Cleaning build directory",
            cwd=build_path
        )
        
        # Configure with all features
        success, output = self.run_command(
            ['cmake', '..', '-DBUILD_TESTS=ON', '-DBUILD_BENCHMARKS=ON'],
            "Configuring CMake",
            cwd=build_path
        )
        
        if not success:
            self.results['tests']['build_configure'] = False
            return False
            
        # Build
        success, output = self.run_command(
            ['cmake', '--build', '.', '-j4'],
            "Building project",
            cwd=build_path
        )
        
        self.results['tests']['build'] = success
        
        # Check executables exist
        executables = ['stn_demo', 'stn_tests', 'stn_benchmark']
        for exe in executables:
            exe_path = build_path / exe
            exists = exe_path.exists()
            print(f"  {'✓' if exists else '✗'} {exe}: {'Found' if exists else 'Not found'}")
            self.results['tests'][f'executable_{exe}'] = exists
            
        return success
        
    def test_simulation_pipeline(self):
        """Test the simulation and navigation pipeline"""
        self.print_header("SIMULATION PIPELINE TEST")
        
        # Generate IMU data
        success, output = self.run_command(
            ['python3', 'python/sim/run_sim.py'],
            "Generating IMU data"
        )
        self.results['tests']['generate_imu'] = success
        
        if not success:
            return False
            
        # Add radar altimeter
        success, output = self.run_command(
            ['python3', 'python/sim/add_radalt.py'],
            "Adding radar altimeter"
        )
        self.results['tests']['add_radalt'] = success
        
        if not success:
            return False
            
        # Run navigation
        exe = self.base_path / 'build' / 'stn_demo'
        if not exe.exists():
            print("❌ Navigation executable not found")
            self.results['tests']['navigation'] = False
            return False
            
        success, output = self.run_command(
            [str(exe), 'data/sim_imu.csv', 'data/run_output.csv', 'data/radalt.csv'],
            "Running navigation"
        )
        self.results['tests']['navigation'] = success
        
        # Parse TRN updates
        if success and "TRN updates:" in output:
            for line in output.split('\n'):
                if "TRN updates:" in line:
                    updates = int(line.split(':')[1].strip().split(')')[0])
                    print(f"  ℹ️ TRN Updates: {updates}")
                    self.results['tests']['trn_updates'] = updates
                    
        return success
        
    def test_grade_performance(self):
        """Test performance grading"""
        self.print_header("PERFORMANCE GRADING TEST")
        
        success, output = self.run_command(
            ['python3', 'python/analysis/grade_check.py'],
            "Running grade check"
        )
        
        self.results['tests']['grade_check'] = success
        
        if success:
            # Parse metrics
            metrics = {}
            for line in output.split('\n'):
                if "GRADE:" in line:
                    grade = line.split("GRADE:")[1].strip()
                    metrics['grade'] = grade
                    print(f"  📊 Grade: {grade}")
                elif "CEP95:" in line:
                    cep = float(line.split(':')[1].strip().split('m')[0])
                    metrics['cep95'] = cep
                    print(f"  📊 CEP95: {cep:.1f}m")
                elif "Final 2D error:" in line:
                    err = float(line.split(':')[1].strip().split('m')[0])
                    metrics['final_error'] = err
                    print(f"  📊 Final Error: {err:.1f}m")
                    
            self.results['tests']['metrics'] = metrics
            
            # Check if meets requirements
            if 'grade' in metrics:
                grade_good = metrics['grade'].startswith('A')
                self.results['tests']['grade_requirement'] = grade_good
                print(f"  {'✅' if grade_good else '⚠️'} Grade requirement: {'PASSED' if grade_good else 'NEEDS IMPROVEMENT'}")
                
        return success
        
    def test_unit_tests(self):
        """Run unit tests"""
        self.print_header("UNIT TESTS")
        
        exe = self.base_path / 'build' / 'stn_tests'
        if not exe.exists():
            print("❌ Test executable not found")
            self.results['tests']['unit_tests'] = False
            return False
            
        success, output = self.run_command(
            [str(exe)],
            "Running unit tests"
        )
        
        self.results['tests']['unit_tests'] = success
        
        # Parse test results
        if success:
            passed = 0
            failed = 0
            
            for line in output.split('\n'):
                if '[  PASSED  ]' in line:
                    passed += 1
                elif '[  FAILED  ]' in line:
                    failed += 1
                    
            total = passed + failed
            if total > 0:
                print(f"  📊 Tests: {passed}/{total} passed ({passed/total*100:.1f}%)")
                self.results['tests']['unit_test_stats'] = {
                    'passed': passed,
                    'failed': failed,
                    'total': total
                }
                
        return success
        
    def test_benchmarks(self):
        """Run performance benchmarks"""
        self.print_header("PERFORMANCE BENCHMARKS")
        
        exe = self.base_path / 'build' / 'stn_benchmark'
        if not exe.exists():
            print("⚠️ Benchmark executable not found (optional)")
            self.results['tests']['benchmarks'] = None
            return True  # Not critical
            
        success, output = self.run_command(
            [str(exe)],
            "Running benchmarks"
        )
        
        self.results['tests']['benchmarks'] = success
        
        # Parse benchmark results
        if success:
            benchmarks = []
            for line in output.split('\n'):
                if 'μs' in line or 'ms' in line:
                    print(f"  ⚡ {line.strip()}")
                    parts = line.strip().split(':')
                    if len(parts) == 2:
                        benchmarks.append({
                            'name': parts[0].strip(),
                            'time': parts[1].strip()
                        })
                        
            self.results['tests']['benchmark_results'] = benchmarks
            
        return True  # Not critical
        
    def test_monte_carlo(self):
        """Run Monte Carlo simulation"""
        self.print_header("MONTE CARLO SIMULATION")
        
        success, output = self.run_command(
            ['python3', 'python/analysis/monte_carlo.py', '--runs', '10'],
            "Running Monte Carlo (10 runs)"
        )
        
        self.results['tests']['monte_carlo'] = success
        
        if success:
            # Parse results
            for line in output.split('\n'):
                if "Mean position error:" in line:
                    mean = float(line.split(':')[1].strip().split('m')[0])
                    print(f"  📊 Mean Error: {mean:.2f}m")
                    self.results['tests']['mc_mean_error'] = mean
                elif "Std deviation:" in line:
                    std = float(line.split(':')[1].strip().split('m')[0])
                    print(f"  📊 Std Dev: {std:.2f}m")
                    self.results['tests']['mc_std_dev'] = std
                    
        return True  # Not critical
        
    def test_data_integrity(self):
        """Verify data file integrity"""
        self.print_header("DATA INTEGRITY CHECKS")
        
        required_files = [
            'data/sim_imu.csv',
            'data/sim_truth.csv',
            'data/radalt.csv',
            'data/run_output.csv'
        ]
        
        all_good = True
        for file_path in required_files:
            full_path = self.base_path / file_path
            exists = full_path.exists()
            
            if exists:
                # Check file size and basic validity
                size = full_path.stat().st_size
                
                if file_path.endswith('.csv'):
                    try:
                        df = pd.read_csv(full_path)
                        rows = len(df)
                        cols = len(df.columns)
                        print(f"  ✓ {file_path}: {rows} rows, {cols} columns, {size/1024:.1f}KB")
                        self.results['tests'][f'data_{Path(file_path).stem}'] = True
                    except Exception as e:
                        print(f"  ✗ {file_path}: Invalid CSV - {str(e)}")
                        self.results['tests'][f'data_{Path(file_path).stem}'] = False
                        all_good = False
            else:
                print(f"  ✗ {file_path}: Not found")
                self.results['tests'][f'data_{Path(file_path).stem}'] = False
                all_good = False
                
        return all_good
        
    def test_configuration(self):
        """Test configuration files"""
        self.print_header("CONFIGURATION TESTS")
        
        config_files = [
            'config/stn_default.cfg',
            'config/stn_realworld.cfg'
        ]
        
        all_good = True
        for config_path in config_files:
            full_path = self.base_path / config_path
            exists = full_path.exists()
            
            if exists:
                size = full_path.stat().st_size
                print(f"  ✓ {config_path}: {size} bytes")
                self.results['tests'][f'config_{Path(config_path).stem}'] = True
            else:
                print(f"  ℹ️ {config_path}: Not found (optional)")
                self.results['tests'][f'config_{Path(config_path).stem}'] = None
                
        return True  # Configs are optional
        
    def generate_report(self):
        """Generate final test report"""
        self.print_header("TEST SUMMARY REPORT")
        
        # Count results
        passed = sum(1 for v in self.results['tests'].values() 
                    if v is True or (isinstance(v, dict) and v) or isinstance(v, (int, float)))
        failed = sum(1 for v in self.results['tests'].values() if v is False)
        skipped = sum(1 for v in self.results['tests'].values() if v is None)
        total = passed + failed + skipped
        
        self.results['summary'] = {
            'passed': passed,
            'failed': failed,
            'skipped': skipped,
            'total': total,
            'success_rate': passed / (passed + failed) * 100 if (passed + failed) > 0 else 0
        }
        
        print(f"\n📊 Results:")
        print(f"  ✅ Passed:  {passed}/{total}")
        print(f"  ❌ Failed:  {failed}/{total}")
        print(f"  ⏭️ Skipped: {skipped}/{total}")
        print(f"  📈 Success Rate: {self.results['summary']['success_rate']:.1f}%")
        
        # Check critical requirements
        print(f"\n🎯 Critical Requirements:")
        
        critical_checks = [
            ('Build System', self.results['tests'].get('build', False)),
            ('Navigation Pipeline', self.results['tests'].get('navigation', False)),
            ('Performance Grade', self.results['tests'].get('grade_requirement', False)),
            ('Unit Tests', self.results['tests'].get('unit_tests', False))
        ]
        
        all_critical_pass = True
        for name, status in critical_checks:
            symbol = '✅' if status else '❌'
            print(f"  {symbol} {name}: {'PASS' if status else 'FAIL'}")
            if not status:
                all_critical_pass = False
                
        # Overall verdict
        print(f"\n{'='*60}")
        if all_critical_pass and self.results['summary']['success_rate'] >= 80:
            print("🎉 SYSTEM STATUS: FULLY OPERATIONAL")
            verdict = "PASS"
        elif self.results['summary']['success_rate'] >= 60:
            print("⚠️ SYSTEM STATUS: OPERATIONAL WITH WARNINGS")
            verdict = "WARNING"
        else:
            print("❌ SYSTEM STATUS: NEEDS ATTENTION")
            verdict = "FAIL"
            
        self.results['summary']['verdict'] = verdict
        
        # Save report
        report_path = self.base_path / 'test_report.json'
        with open(report_path, 'w') as f:
            json.dump(self.results, f, indent=2, default=str)
        print(f"\n💾 Detailed report saved to: {report_path}")
        
        return verdict
        
    def run_all_tests(self):
        """Run complete test suite"""
        print("\n" + "🚀 STN NAVIGATION SYSTEM - COMPREHENSIVE TEST SUITE 🚀".center(60))
        print("="*60)
        
        start_time = time.time()
        
        # Run test sequence
        test_sequence = [
            ('Build System', self.test_build_system),
            ('Simulation Pipeline', self.test_simulation_pipeline),
            ('Performance Grading', self.test_grade_performance),
            ('Unit Tests', self.test_unit_tests),
            ('Benchmarks', self.test_benchmarks),
            ('Data Integrity', self.test_data_integrity),
            ('Configuration', self.test_configuration),
            ('Monte Carlo', self.test_monte_carlo)
        ]
        
        for test_name, test_func in test_sequence:
            try:
                test_func()
            except Exception as e:
                print(f"❌ {test_name} failed with exception: {str(e)}")
                self.results['tests'][test_name.lower().replace(' ', '_')] = False
                
        # Generate report
        verdict = self.generate_report()
        
        elapsed = time.time() - start_time
        print(f"\n⏱️ Total test time: {elapsed:.1f} seconds")
        print("="*60)
        
        # Return exit code based on verdict
        if verdict == "PASS":
            return 0
        elif verdict == "WARNING":
            return 1
        else:
            return 2


if __name__ == "__main__":
    runner = TestRunner()
    exit_code = runner.run_all_tests()
    sys.exit(exit_code)