import os
import shutil
import subprocess
import sys
import time

# --- Configuration ---
# Name of the conda environment where dependencies are installed
CONDA_ENV_NAME = "roboos"

# Get the directory where this script is located to make paths robust
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROBOBRAIN_PATH = os.path.join(SCRIPT_DIR, "RoboBrain2.0")
ROBOOS_PATH = os.path.join(SCRIPT_DIR, "RoboOS")

def launch_in_new_terminal(command, title):
    """Launches a command in a new gnome-terminal window."""
    # Using 'bash -c' allows us to chain commands.
    # 'exec bash' keeps the terminal open after the command finishes for inspection.
    full_command = f'gnome-terminal --tab --title="{title}" -- /bin/bash -c "{command}; exec bash"'
    print(f"🚀 Launching: {title}")
    print(f"   Command: {command}")
    try:
        subprocess.Popen(full_command, shell=True)
    except Exception as e:
        print(f"❌ Failed to launch {title}. Error: {e}")
        print(
            "   Please ensure 'gnome-terminal' is installed or modify the script for your terminal emulator."
        )


def main():
    """Main function to check prerequisites and launch all components."""
    print("========================================")
    print(" RoboOS System Launcher")
    print("========================================")

    print("\nPreparing and Launching commands...")

    commands = [
        # {"title": "Message Broker (Redis)", "command": "killall redis-server > /dev/null 2>&1 || true; redis-server"},
        {"title": "LLM Server (RoboBrain 2.0)", "command": f"cd {ROBOBRAIN_PATH} && python robobrain_server.py"},
        {"title": "RoboOS Master", "command": f"cd {ROBOOS_PATH} && python master/run.py"},
        {"title": "RoboOS Slaver", "command": f"cd {ROBOOS_PATH} && python slaver/run.py"},
    ]
    
    for component in commands:
        launch_in_new_terminal(component["command"], component["title"])
        time.sleep(2)  # Stagger the launches slightly

    print("\n✅ All components launched. Please check the new terminal windows for status and logs.")


if __name__ == "__main__":
    main()
