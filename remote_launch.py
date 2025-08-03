#!/usr/bin/env python3
import os
import subprocess
import time
import argparse
import webbrowser

# This script is designed to launch the RoboOS (Master) and RoboBrain2.0 system components in separate terminal windows on the remote server.

# --- Configuration ---
# Name of the conda environment where dependencies are installed
CONDA_ENV_NAME = "roboos"

# Get the directory where this script is located to make paths robust
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOBRAIN_PATH = os.path.join(SCRIPT_DIR, "RoboBrain2.0")
ROBOOS_PATH = os.path.join(SCRIPT_DIR, "RoboOS")


def parse_args():
    """Parses command-line arguments."""
    parser = argparse.ArgumentParser(description="Launch RoboOS and RoboBrain2.0 system components.")
    parser.add_argument("-d", "--debug", action="store_true", default=False, help="Keep terminal open after command finishes (debug mode)",)
    return parser.parse_args()


def launch_in_new_terminal(command, title, debug=False):
    """Launches a command in a new gnome-terminal window."""
    # 'exec bash' keeps the terminal open after the command finishes for inspection (debug mode).
    exec_bash = "; exec bash" if debug else ""
    full_command = (f'gnome-terminal --tab --title="{title}" -- /bin/bash -c "{command}{exec_bash}"')
    print(f"🚀 Launching: {title}")
    print(f"   Command: {command}")
    subprocess.Popen(full_command, shell=True)

def is_redis_running():
    """Checks if redis-server is already running."""
    try:
        # Use pgrep to check for the process. -x ensures an exact match.
        subprocess.run(['pgrep', '-x', 'redis-server'], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        # CalledProcessError means pgrep ran but found no process (exit code 1)
        # FileNotFoundError means pgrep is not installed
        return False

def main():
    """Main function to check prerequisites and launch all components."""
    args = parse_args()

    print("========================================")
    print(" RoboOS and RoboBrain2.0 System Launcher")
    print("========================================")

    print("\nPreparing and Launching commands...")

    commands = [
        {"title": "LLM Server (RoboBrain 2.0)", "command": f"cd {ROBOBRAIN_PATH} && python robobrain_server.py"},
        {"title": "RoboOS Master", "command": f"cd {ROBOOS_PATH} && python master/run.py"},
        {"title": "Gradio UI", "command": f"cd {ROBOOS_PATH} && python gradio_ui.py"},
    ]

    # Check if Redis needs to be launched
    if not is_redis_running():
        print("Redis server not found, launching it...")
        # Note: This assumes redis.conf is in the SCRIPT_DIR, which is the root of the project.
        redis_conf_path = os.path.join(SCRIPT_DIR, 'redis.conf')
        commands.insert(0, {"title": "Message Broker (Redis)", "command": f"redis-server {redis_conf_path}"})
    else:
        print("✅ Redis server is already running.")


    for component in commands:
        launch_in_new_terminal(component["command"], component["title"], debug=args.debug)
        time.sleep(2)  # Stagger the launches slightly

    print("\n✅ All components launched. Please check the new terminal windows for status and logs.")
    print("Opening Gradio UI in your browser...")
    # Give the Gradio server a moment to start
    time.sleep(5)
    webbrowser.open("http://127.0.0.1:7860")


if __name__ == "__main__":
    main()
