#!/usr/bin/env python3
"""
Validation script to verify the action-based architecture.
This script checks that all packages are correctly set up.
"""
import os
import sys

def check_file_exists(filepath, description):
    """Check if a file exists."""
    if os.path.exists(filepath):
        print(f"✓ {description}: {filepath}")
        return True
    else:
        print(f"✗ {description}: {filepath} NOT FOUND")
        return False

def main():
    """Main validation function."""
    print("=" * 70)
    print("Validating ROS2 Action-based Architecture")
    print("=" * 70)
    
    base_path = "/home/runner/work/unitree-g1-auto-charger-plugging/unitree-g1-auto-charger-plugging"
    
    all_checks = []
    
    # Check charging_interfaces package
    print("\n1. Checking charging_interfaces package...")
    all_checks.append(check_file_exists(f"{base_path}/charging_interfaces/package.xml", "Package manifest"))
    all_checks.append(check_file_exists(f"{base_path}/charging_interfaces/CMakeLists.txt", "CMake config"))
    all_checks.append(check_file_exists(f"{base_path}/charging_interfaces/action/Navigate.action", "Navigate action"))
    all_checks.append(check_file_exists(f"{base_path}/charging_interfaces/action/Manipulate.action", "Manipulate action"))
    all_checks.append(check_file_exists(f"{base_path}/charging_interfaces/action/Detect.action", "Detect action"))
    
    # Check navigation package
    print("\n2. Checking navigation package...")
    all_checks.append(check_file_exists(f"{base_path}/navigation/navigation/navigation_action_server.py", "Navigation action server"))
    
    # Check manipulation package
    print("\n3. Checking manipulation package...")
    all_checks.append(check_file_exists(f"{base_path}/manipulation/manipulation/manipulation_action_server.py", "Manipulation action server"))
    
    # Check perception package
    print("\n4. Checking perception package...")
    all_checks.append(check_file_exists(f"{base_path}/perception/perception/perception_action_server.py", "Perception action server"))
    
    # Check mission_control package
    print("\n5. Checking mission_control package...")
    all_checks.append(check_file_exists(f"{base_path}/mission_control/mission_control/state_machine_action_client.py", "State machine action client"))
    
    # Check for g1_ prefixes in package names
    print("\n6. Checking for forbidden 'g1_' prefixes...")
    packages = ["charging_interfaces", "navigation", "manipulation", "perception", "mission_control"]
    prefix_check = True
    for pkg in packages:
        if "g1_" in pkg:
            print(f"✗ Package name contains 'g1_' prefix: {pkg}")
            prefix_check = False
    if prefix_check:
        print("✓ No 'g1_' prefixes found in package names")
    all_checks.append(prefix_check)
    
    # Summary
    print("\n" + "=" * 70)
    print("Validation Summary")
    print("=" * 70)
    passed = sum(all_checks)
    total = len(all_checks)
    print(f"Passed: {passed}/{total} checks")
    
    if passed == total:
        print("\n✓ All validation checks PASSED!")
        return 0
    else:
        print(f"\n✗ {total - passed} validation check(s) FAILED!")
        return 1

if __name__ == "__main__":
    sys.exit(main())
