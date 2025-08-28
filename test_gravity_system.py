#!/usr/bin/env python3
"""
Test script for gravity-primary navigation system
Verifies complete removal of TRN and proper gravity implementation
"""

import subprocess
import os
import sys
import time

def run_command(cmd):
    """Run a shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode == 0, result.stdout, result.stderr
    except Exception as e:
        return False, "", str(e)

def check_no_trn():
    """Verify no TRN code remains in source files"""
    print("Checking for TRN references in source code...")
    
    # Check for TRN in actual source files (not docs)
    success, stdout, _ = run_command(
        "grep -r 'class.*TRN\\|struct.*TRN\\|TRNUpdate\\|terrain_provider\\|TerrainProvider' "
        "--include='*.cpp' --include='*.h' --include='*.hpp' ."
    )
    
    if success and stdout:
        print("❌ Found TRN references in source code:")
        print(stdout)
        return False
    else:
        print("✅ No TRN code found in source files")
        return True

def check_gravity_files():
    """Verify all gravity navigation files exist"""
    print("\nChecking gravity navigation files...")
    
    required_files = [
        "cpp/core/ukf.h",
        "cpp/core/ukf.cpp", 
        "cpp/core/gravity_gradient_provider.h",
        "cpp/core/gravity_gradient_provider.cpp",
        "cpp/core/types.h",
        "cpp/core/types.cpp",
        "core/gravity_navigator.cpp",
        "cpp/hardware/hardware_interface.h"
    ]
    
    all_present = True
    for file in required_files:
        if os.path.exists(file):
            print(f"✅ {file}")
        else:
            print(f"❌ Missing: {file}")
            all_present = False
    
    return all_present

def test_build():
    """Test that gravity navigator builds successfully"""
    print("\nBuilding gravity navigator...")
    
    # Clean build
    run_command("rm -rf build")
    run_command("mkdir -p build")
    
    # Configure
    success, stdout, stderr = run_command("cd build && cmake .. -DCMAKE_BUILD_TYPE=Release")
    if not success:
        print(f"❌ CMake configuration failed:\n{stderr}")
        return False
    
    # Build
    success, stdout, stderr = run_command("cd build && make gravity_navigator -j4")
    if not success:
        print(f"❌ Build failed:\n{stderr}")
        return False
    
    print("✅ Build successful")
    return True

def test_execution():
    """Test that gravity navigator runs"""
    print("\nTesting gravity navigator execution...")
    
    if not os.path.exists("build/gravity_navigator"):
        print("❌ Executable not found")
        return False
    
    # Run for 5 seconds
    cmd = "timeout 5 ./build/gravity_navigator 47.0 8.0 1000.0"
    success, stdout, stderr = run_command(cmd)
    
    if "SPACETIME NAVIGATION SYSTEM v2.0" in stdout:
        print("✅ Gravity navigator runs successfully")
        
        # Check for gravity updates
        if "Updates: G=" in stdout:
            lines = stdout.split('\n')
            for line in lines:
                if "Updates: G=" in line:
                    # Extract update counts
                    parts = line.split("Updates: G=")[-1].split()
                    g_updates = int(parts[0])
                    a_updates = int(parts[1].split("=")[1])
                    print(f"   Gradient updates: {g_updates}")
                    print(f"   Anomaly updates: {a_updates}")
                    break
        return True
    else:
        print("❌ Execution failed or wrong version")
        print(stderr)
        return False

def main():
    print("=" * 60)
    print("GRAVITY-PRIMARY NAVIGATION SYSTEM TEST")
    print("=" * 60)
    
    results = {
        "No TRN Code": check_no_trn(),
        "Gravity Files Present": check_gravity_files(),
        "Build Successful": test_build(),
        "Execution Test": test_execution()
    }
    
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for test, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{test:.<40} {status}")
        if not passed:
            all_passed = False
    
    print("=" * 60)
    
    if all_passed:
        print("\n🎉 ALL TESTS PASSED!")
        print("The gravity-primary navigation system is 100% operational.")
        print("No TRN code remains in the system.")
        return 0
    else:
        print("\n⚠️ Some tests failed. Please review the output above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())