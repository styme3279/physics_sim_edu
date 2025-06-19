#!/usr/bin/env python3
"""
Test runner script for physics_simulator.
Provides convenient commands for running different types of tests.
"""
import sys
import subprocess
import argparse
from pathlib import Path


def run_command(cmd, description):
    """Run a command and handle errors."""
    print(f"\n{'='*60}")
    print(f"Running: {description}")
    print(f"Command: {' '.join(cmd)}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, check=True, capture_output=False)
        print(f"✅ {description} completed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ {description} failed with exit code {e.returncode}")
        return False
    except FileNotFoundError:
        print(f"❌ Command not found. Make sure pytest is installed: pip install pytest")
        return False


def main():
    parser = argparse.ArgumentParser(description="Run physics_simulator tests")
    parser.add_argument(
        "--type", 
        choices=["unit", "integration", "all", "coverage"], 
        default="all",
        help="Type of tests to run"
    )
    parser.add_argument(
        "--verbose", "-v", 
        action="store_true", 
        help="Verbose output"
    )
    parser.add_argument(
        "--fast", 
        action="store_true", 
        help="Skip slow tests"
    )
    
    args = parser.parse_args()
    
    # Base pytest command
    base_cmd = ["python", "-m", "pytest"]
    
    if args.verbose:
        base_cmd.append("-v")
    
    # Test type specific commands
    if args.type == "unit":
        cmd = base_cmd + ["-m", "unit", "tests/"]
        description = "Unit Tests"
    elif args.type == "integration":
        cmd = base_cmd + ["-m", "integration", "tests/"]
        description = "Integration Tests"
    elif args.type == "coverage":
        cmd = base_cmd + ["--cov=src/physics_simulator", "--cov-report=html", "tests/"]
        description = "Coverage Tests"
    else:  # all
        cmd = base_cmd + ["tests/"]
        description = "All Tests"
        
    if args.fast:
        cmd.extend(["-m", "not slow"])
    
    # Run the tests
    success = run_command(cmd, description)
    
    if args.type == "coverage" and success:
        print(f"\n📊 Coverage report generated in: htmlcov/index.html")
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main()) 