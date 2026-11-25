"""BACS Reproducibility Environment Verification Script.

Checks system dependencies, Python libraries, data paths, and ROS 2 environment.
"""

import sys
import os
import shutil

REQUIRED_MODULES = [
    "numpy",
    "scipy",
    "matplotlib",
    "pandas",
]

OPTIONAL_MODULES = [
    "gtsam",
    "serial",
    "openpyxl",
    "docx",
]


def check_python_version():
    v = sys.version_info
    print(f"[*] Python version: {v.major}.{v.minor}.{v.micro}")
    if v.major != 3 or v.minor < 10:
        print("[!] Warning: Python 3.10+ is recommended.")
        return False
    return True


def check_python_modules():
    all_ok = True
    print("[*] Checking required Python packages:")
    for mod in REQUIRED_MODULES:
        try:
            __import__(mod)
            print(f"  [+] {mod}: OK")
        except ImportError:
            print(f"  [-] {mod}: MISSING")
            all_ok = False

    print("[*] Checking optional/embedded packages:")
    for mod in OPTIONAL_MODULES:
        try:
            __import__(mod)
            print(f"  [+] {mod}: OK")
        except ImportError:
            print(f"  [ ] {mod}: not installed (optional)")
    return all_ok


def check_ros2_environment():
    print("[*] Checking ROS 2 environment:")
    ros_distro = os.environ.get("ROS_DISTRO", None)
    if ros_distro:
        print(f"  [+] Active ROS 2 Distribution: {ros_distro}")
    else:
        print("  [ ] No active ROS_DISTRO detected in current shell (source /opt/ros/humble/setup.bash for ROS 2 nodes).")

    colcon_path = shutil.which("colcon")
    if colcon_path:
        print(f"  [+] colcon build tool: {colcon_path}")
    else:
        print("  [ ] colcon not found in PATH.")


def check_repository_paths():
    print("[*] Checking key repository directories:")
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    expected_dirs = [
        "bacs_sim",
        "config/sim",
        "runs/v15",
        "reviewer_R1_3_geometry_outages",
        "scripts",
        "tests",
        "algorithms",
        "analysis",
        "ros2_ws",
        "hardware",
    ]
    for d in expected_dirs:
        p = os.path.join(repo_root, d)
        if os.path.isdir(p):
            print(f"  [+] {d}/: Found")
        else:
            print(f"  [-] {d}/: MISSING")


if __name__ == "__main__":
    print("=" * 60)
    print(" BACS Scientific Reproducibility Environment Verification")
    print("=" * 60)
    py_ok = check_python_version()
    mod_ok = check_python_modules()
    check_ros2_environment()
    check_repository_paths()
    print("=" * 60)
    if py_ok and mod_ok:
        print("[SUCCESS] Core environment is ready for simulation and analysis.")
    else:
        print("[NOTICE] Please install missing dependencies before running full pipeline.")
    print("=" * 60)
