# Step-by-Step Installation and Setup

### Step 1: System Prerequisites
First, let's set up your environment.

```bash
# Update your package list
sudo apt update

# Install Redis Server
sudo apt install redis-server

# Start Redis and enable it to run on boot
sudo systemctl start redis-server
sudo systemctl enable redis-server

# Check if Redis is running
# You should see "active (running)" in the output
sudo systemctl status redis
```

### Step 2: Create a Conda Environment
It's best practice to keep project dependencies isolated.

```bash
# Create a new conda environment with Python 3.10
conda create --name roboos python=3.10 -y

# Activate the environment
conda activate roboos
```

### Step 3: Install RoboOS Dependencies

```bash
# Clone the repository
git clone https://github.com/FlagOpen/RoboOS.git
cd RoboOS

# Install the required Python packages
pip install -r requirements.txt

# You will need a Large Language Model. Let's install what's needed for a 7B model.
# The Hugging Face Transformers library is a common choice.
pip install transformers torch accelerate bitsandbytes

# If you want to use the Hugging Face Hub, ensure you have the latest version
pip install --upgrade huggingface_hub
huggingface-cli login
```

### Step 4: Configure the LLM (Simulating RoboBrain 2.0)
```bash
```

## Communication: MCP and ROS2 Integration

### Step 1: Add the ROS2 Python path to your conda environment's paths

```bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
# Or
conda install conda-build
conda develop /opt/ros/humble/lib/python3.10/site-packages
```

### Step 2: Modify the Slaver to be a ROS2 Node
```bash
```
Simulation Platforms:
[AWS RoboMaker hospital world](https://github.com/aws-robotics/aws-robomaker-hospital-world): AWS provides a sample hospital world for Gazebo, which is perfect for ROS2 integration.

## Final Launch Procedure

### Terminal 1: Start Redis (if not already running)
```bash
redis-server
```

### Terminal 2: Launch TurtleBot3 Simulation & Nav2
```bash
# Source your ROS2 workspace
source ~/turtlebot3_ws/install/setup.bash

# Export the TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Launch the simulation with the Nav2 stack
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/path/to/your/map.yaml
```

### Terminal 3: Start the RoboOS Master
```bash
# Activate conda environment
conda activate roboos_env
cd /path/to/RoboOS

# This will load the LLM and wait for slaver connections
python master/run.py 
```

### Terminal 4: Start the RoboOS Slaver (ROS2 Bridge)
```bash
# Activate conda environment
conda activate roboos_env
cd /path/to/RoboOS

# This script now connects to Redis and ROS2
python slaver/run.py
```

### Terminal 5: Send the Initial Command
You would use another script or the gradio_ui.py to send the initial message to the Master. For testing, you can use a simple Redis client:

```python
import redis
import json
r = redis.Redis(host='localhost', port=6379, db=0)
user_query = {"user_id": "patient_123", "text": "I have a stomachache."}
r.publish('user_queries_channel', json.dumps(user_query)) # Master must subscribe to this channel
```
