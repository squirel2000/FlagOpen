#!/usr/bin/env python3
import os
import subprocess
import time
import argparse

# This script is designed to launch the local ROS2 components for the robot.

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOOS_PATH = os.path.join(SCRIPT_DIR, "RoboOS")
MAP_PATH = os.path.join(SCRIPT_DIR, "map.yaml")

def parse_args():
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(description="Launch local ROS2 components.")
    parser.add_argument("-t", "--teleop", action="store_true", default=False, help="Launch teleop_keyboard")
    parser.add_argument("-s", "--slam", action="store_true", default=False, help="Launch SLAM")
    parser.add_argument("-n", "--nav", action="store_true", default=False, help="Launch Navigation")
    parser.add_argument("-d", "--debug", action="store_true", default=False, help="Keep terminal open after command finishes (debug mode)")
    return parser.parse_args()

def launch_in_new_terminal(command, title, debug=False):
    """Launches a command in a new gnome-terminal window."""
    # 'exec bash' keeps the terminal open after the command finishes for inspection (debug mode).
    exec_bash = "; exec bash" if debug else ""
    full_command = (f'gnome-terminal --tab --title="{title}" -- /bin/bash -c "{command}{exec_bash}"')
    print(f"🚀 Launching: {title}")
    print(f"   Command: {command}")
    subprocess.Popen(full_command, shell=True)

def main():
    """Main function to check prerequisites and launch all components."""
    args = parse_args()

    print("=========================================")
    print("        Local ROS2 System Launcher       ")
    print("=========================================")

    print("\nPreparing and Launching commands...")

    commands = [
        {"title": "RoboOS Slaver", "command": f"source /home/asus/miniforge3/etc/profile.d/conda.sh && conda activate roboos && cd {ROBOOS_PATH} && python slaver/run.py"},
        {"title": "Gazebo", "command": "ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py"},
    ]

    if not args.nav:  # nav2 has included the RViz.
        commands.append({"title": "RViz", "command": "ros2 launch turtlebot3_bringup rviz2.launch.py"})
    if args.teleop:
        commands.append({"title": "Teleop Keyboard", "command": "ros2 run turtlebot3_teleop teleop_keyboard"})
    if args.slam:
        commands.append({"title": "SLAM", "command": "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"})
    if args.nav:
        commands.append({"title": "Navigation", "command": f"ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:={MAP_PATH}"})

    for component in commands:
        launch_in_new_terminal(component["command"], component["title"], debug=args.debug)
        time.sleep(2)  # Stagger the launches slightly

    print("\n✅ All components launched. Please check the new terminal windows for status and logs.")


if __name__ == "__main__":
    main()
