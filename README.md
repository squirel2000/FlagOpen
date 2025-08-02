# Instruction for RoboOS and RoboBrain 2.0


## Step-by-Step Installation and Setup

### Step 1: System Prerequisites
First, let's set up your environment.

```bash
# Update your package list
sudo apt update

# Install Redis Server
sudo apt install redis-server -y

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
git clone https://github.com/squirel2000/RoboOS.git
cd RoboOS

# Install the required Python packages
pip install -r requirements.txt

# You will need a Large Language Model. Let's install what's needed for a 7B model.
# The Hugging Face Transformers library is a common choice.
pip install transformers torch accelerate bitsandbytes

# If you want to use the Hugging Face Hub, ensure you have the latest version
pip install --upgrade huggingface_hub
huggingface-cli login

# Token for the RoboBrain:
#   HF_TOKEN is exported in ~/.bashrc
# 
# Token is valid (permission: fineGrained).
# The token `RoboBrain` has been saved to /home/asus/.cache/huggingface/stored_tokens
# Your token has been saved in your configured git credential helpers (store).
# Your token has been saved to /home/asus/.cache/huggingface/token
# Login successful.
```

### Step 4: Configure the LLM (Simulating RoboBrain 2.0)
```bash
# Clone the repository
git clone https://github.com/squirel2000/RoboBrain2.0.git
cd RoboBrain2.0

# Install the required Python packages
pip install -r requirements.txt
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

### On the local machine:
```bash
conda activate roboos
# Launch the simultion environmnet
# 1. TurtleBot3 Gazebo simulation (with TURTLEBOT3_MODEL=waffle) and RViz
# 2. teleop, cartographer SLAM or Nav2 Stack
# 3. RoboOS Slaver (ROS2 Bridge)
python3 local_launch.py
```

### On the remote server:
```bash
# Activate conda environment
conda activate roboos
# Start Redis, RoboOS Master, and RoboBrain 2.0 with the debug mode
python3 remote_launch.py -d
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


# Problem Solving and Debugging

Please respond as a professional and pratical RoboOS engineer to a beginner. 

I want to study, set up, and implement the RoboOS in https://github.com/FlagOpen/RoboOS, so I have cloned the RoboOS, RoboBrain 2.0, and RoboSkill. 

Let's take the following hospital scenario for example, a. Someone talks/texts to a robot (A) with a message "I have a stomachache." b. The robot (A) sends this information to the RoboOS on Cloud (via MQTT or other protocol). c. RoboOS analyzes and has the result of "Bring the person to room 001". (The doctor in room 001 has proficiency in Gastroenterology) Do I need to create a tool list for the RoboOS to train/find-tune the model or create a database? How to let the RoboOS know it should generate the response something like "Bring the patient to room 001" d. RoboOS sends tasks to another mobile robot (B) (I am not sure if the RoboBrain should be installed on this robot?). What's the communication protocol is suitable or recommended? Is the MCP (Model context Protocol) suitable in this case? If yes, how do I set up the environment including necessary dependencies? e. The robot (B) decides and generates sequential tasks, such as 1. move to the front desk to meet the customer, (using the navigation function in ROS2) 2. guide the customer to room 001, 3. Respond a finished message to the RoboOS (If any exception, the robot will be terminated and respond to the RoboOS.) 

1. If I want to set up the RoboOS with RoboBrain 2.0 (prefer to 7B parameter LLM due to the limited vRAM) from the beginning and finally launch the hospital scenario example/sample, please provide the complete instructions step-by-step. (The environment is Ubuntu 22.04 with Python 3.10, and the Conda and ROS2 Humble with Turtlebot3 package are installed. The computer is equipped with a RTX 4070 graphic card. Or do I have to run the slaver and master on different computers? How to set up the redis-server? Is it free?) If I want to integrate the RoboBrain 2.0 in https://github.com/FlagOpen/RoboBrain2.0, please also provide the instruction in detail. 
2. To set up the hospital sample, what's the training data should be input to the RoboBrain 2.0 or RoboOS for training? How to make the RoboBrain study or train the LLM? Please also provide some actual Sample Tool List (JSON) that I can use directly and the step-by-step procedure to integrate knowledge and tools in RoboOS. Please also search and provide the platforms/datasets/skill templates in your response of "Many robotics/AI platforms provide sample hospital environments, datasets, and skill templates". I create a simple tool list, hospital_tools.json, how to modify the master/run.py in /master/run.py to execute the corresponding python function from hospital_tools.py? 
3. The MCP protocol is selected, but how to set up the ros-mcp-server as you mentioned in "Use ROS2 with an MCP bridge (e.g., ros-mcp-server)"? In the ROS2 side, how to set up the turtlebot3 to receive the message from the MCP bridge? Please also provide a script to install /set up the MCP server/client. 
4. Please modify the slaver/run.py based on the https://github.com/FlagOpen/RoboOS/blob/main/slaver/run.py? (I can use the navigate_to_pose action server or I have another option with a ROS2 node, behavior_coordinator_node.py as attached to call an follow_user_motion_node.py to perform the navigation from the current pose to the target pose, so please base on the simple source file to modify the slaver/run.py from a professional software engineer's side.) 

(The RoboOS, RoboBrain2.0, RoboSkill are cloned in this project, and the Python environment with python=3.0 is created as "roboos" using conda with requirements.txt installed, and the redis-server is also installed. Some tasks has been done by myself.)