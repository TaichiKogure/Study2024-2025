#!/usr/bin/env python3

import os
import sys

def check_file_exists(file_path, file_description):
    if os.path.exists(file_path):
        print(f"✓ {file_description} found: {file_path}")
        return True
    else:
        print(f"✗ {file_description} NOT found: {file_path}")
        return False

def main():
    # Get the current directory (should be the package root)
    package_root = os.path.dirname(os.path.abspath(__file__))
    print(f"Checking package structure in: {package_root}")
    
    # Define the files to check
    files_to_check = [
        (os.path.join(package_root, "package.xml"), "Package manifest"),
        (os.path.join(package_root, "setup.py"), "Setup script"),
        (os.path.join(package_root, "resource", "turtle_simulation"), "Resource marker"),
        (os.path.join(package_root, "turtle_simulation", "__init__.py"), "Python package init"),
        (os.path.join(package_root, "turtle_simulation", "turtle_keyboard_node.py"), "Keyboard controller node"),
        (os.path.join(package_root, "turtle_simulation", "turtle_controller_node.py"), "Turtle controller node"),
        (os.path.join(package_root, "turtle_simulation", "gazebo_turtle_node.py"), "Gazebo turtle node"),
        (os.path.join(package_root, "launch", "turtle_sim.launch.py"), "TurtleSim launch file"),
        (os.path.join(package_root, "launch", "gazebo_sim.launch.py"), "Gazebo launch file"),
        (os.path.join(package_root, "worlds", "turtle_world.world"), "Gazebo world file"),
        (os.path.join(package_root, "models", "turtle.urdf"), "Turtle URDF model"),
        (os.path.join(package_root, "config", "turtle.rviz"), "RViz configuration"),
        (os.path.join(package_root, "README.md"), "User manual")
    ]
    
    # Check each file
    all_files_exist = True
    for file_path, description in files_to_check:
        if not check_file_exists(file_path, description):
            all_files_exist = False
    
    # Print summary
    print("\nSummary:")
    if all_files_exist:
        print("✓ All required files are present. The package structure is correct.")
        print("  You can build the package with:")
        print("  colcon build --symlink-install --packages-select turtle_simulation")
    else:
        print("✗ Some required files are missing. Please check the output above.")
    
    return 0 if all_files_exist else 1

if __name__ == "__main__":
    sys.exit(main())