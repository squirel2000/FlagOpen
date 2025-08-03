# Guideline: RoboOS Hospital Scenario End-to-End Deployment

This document outlines the complete process for setting up and running a hospital assistance scenario using the RoboOS framework, RoboBrain 2.0, and a ROS2-simulated robot.

## 1. Scenario & Architecture

### High-Level Scenario

A person approaches a patient-assistance kiosk (**Robot A**, simulated by our UI) and says, "**I have a stomachache.**"

1.  **Task Ingestion**: The kiosk sends this text to the central **RoboOS Master** server.
2.  **Intelligent Planning**: The **RoboOS Master** uses **RoboBrain 2.0** to understand the request. It consults its knowledge base ([`scene_profile.yaml`](RoboOS/master/profile/scene_profile.yaml)) and determines that a stomachache requires a visit to the Gastroenterology clinic in "Room 001".
3.  **Task Decomposition**: The Master formulates a high-level subtask, such as `guide the patient from the Front Desk to Room 001`.
4.  **Dispatch**: The Master dispatches this subtask to an available mobile robot, **Robot B**.
5.  **Execution**: **Robot B** (our Slaver and ROS2 simulation) receives the command. Its toolset ([`hospital_tools.py`](RoboOS/tools/hospital_tools.py)) translates the symbolic locations ("Front Desk", "Room 001") into map coordinates and executes the navigation using ROS2.

### System Architecture

The system runs across two machines, coordinated by a central Redis message broker.

*   **Master Server (The "Brain" - `192.168.100.2`)**
    *   **Hardware**: Server with RTX 4090.
    *   **Components**:
        *   **Redis Server**: The central message bus.
        *   **RoboBrain 2.0 Server**: Provides LLM-based reasoning.
        *   **RoboOS Master**: Orchestrates the high-level plan.
        *   **Gradio UI**: A user-friendly web interface for sending tasks to the Master.

*   **Slaver Robot (The "Body" - `192.168.100.3`)**
    *   **Hardware**: Computer with RTX 4070.
    *   **Components**:
        *   **ROS2 & Gazebo**: The robotics middleware and simulation environment.
        *   **RoboOS Slaver**: The on-robot agent that executes commands by translating them into ROS2 actions.

## 2. Code Flow & Task Grounding

The core challenge is translating an abstract goal into physical action. Here's how the code handles it:

1.  **Task Initiation (`gradio_ui.py`)**: The user types "I have a stomachache" into the web UI and clicks "Submit". The UI sends a POST request to the Master's `/publish_task` endpoint.
2.  **Master Planning (`master/agents/agent.py`)**: The Master receives the task. It calls the `GlobalTaskPlanner` ([`master/agents/planner.py`](RoboOS/master/agents/planner.py)), which constructs a detailed prompt for RoboBrain, including scene information.
3.  **LLM Inference (`robobrain_server.py`)**: RoboBrain receives the prompt and returns a structured JSON plan. It operates on **symbolic locations** (e.g., "Front Desk").
4.  **Asynchronous Dispatch (`master/agents/agent.py`)**: The Master receives the plan and immediately starts a **background thread** to handle the execution, allowing the UI to respond without delay.
5.  **Subtask Transmission (Redis)**: The background thread sends the planned subtask to the Slaver's channel on the Redis server.
6.  **Slaver Execution (`slaver/run.py`)**: The Slaver agent receives the subtask. It identifies the required tool (e.g., `guide_patient_to_room`) and calls the corresponding function from the `HospitalRos2Bridge`.
7.  **Grounding & ROS2 Action (`tools/hospital_tools.py`)**: This is the critical translation step. The `guide_patient` function in the `HospitalRos2Bridge` takes the symbolic locations ("Front Desk", "Room 001"), looks up their real-world `(x, y, w)` coordinates from an internal dictionary, and sends a `NavigateToPose` goal to the ROS2 navigation stack.
8.  **Robot Action (Gazebo)**: The simulated robot begins moving.

## 3. Step-by-Step Setup & Configuration

#### A. On the Master Server (`192.168.100.2`)

1.  **Install and Configure Redis:**
    ```bash
    sudo apt update && sudo apt install redis-server
    # Allow Redis to accept connections from other computers
    sudo sed -i 's/bind 127.0.0.1 -::1/bind 0.0.0.0/' /etc/redis/redis.conf
    sudo systemctl restart redis-server
    ```

2.  **Download RoboBrain 2.0 Model:**
    ```bash
    # Install Git LFS if you haven't already
    sudo apt-get install git-lfs && git-lfs install
    # Clone the model (this is large and will take time)
    git clone https://huggingface.co/BAAI/RoboBrain-2.0-7B /path/to/your/models/RoboBrain-2.0-7B
    ```

3.  **Configure Project Files:**
    *   **Master Config**: In [`RoboOS/master/config.yaml`](RoboOS/master/config.yaml), set the `HOST` to the server's IP.
        ```yaml
        communicator:
          HOST: "192.168.100.2"
        ```
    *   **RoboBrain Server**: In [`RoboBrain2.0/robobrain_server.py`](RoboBrain2.0/robobrain_server.py), update the model path to your local download.
        ```python
        model_inference = SimpleInference("/path/to/your/models/RoboBrain-2.0-7B")
        ```
    *   **Gradio UI**: In [`RoboOS/gradio_ui.py`](RoboOS/gradio_ui.py), ensure the `MASTER_URL` points to the correct address.
         ```python
        MASTER_URL = "http://192.168.100.2:5001/publish_task"
        ```

#### B. On the Slaver Robot Computer (`192.168.100.3`)

1.  **Install ROS2 & Gazebo:** Ensure you have a working ROS2 Humble and Turtlebot3 simulation environment.

2.  **Configure Project Files:**
    *   **Slaver Config**: In [`RoboOS/slaver/config.yaml`](RoboOS/slaver/config.yaml), set the `HOST` to point to the **Master Server's IP**.
        ```yaml
        communicator:
          HOST: "192.168.100.2"
        ```
    *   **Tool Implementation**: Ensure your [`RoboOS/tools/hospital_tools.py`](RoboOS/tools/hospital_tools.py) contains the `HospitalRos2Bridge` class with coordinate lookups for your symbolic locations.
    *   **Slaver Runner**: Ensure [`RoboOS/slaver/run.py`](RoboOS/slaver/run.py) is the modified version that initializes `rclpy` and the `HospitalRos2Bridge`.

## 4. Running the Full Scenario with Launch Scripts

These modified launch scripts streamline the startup process.

#### A. On the Master Server (`192.168.100.2`)

Run the remote launch script. This will open three new terminal tabs for the Redis server (if not already running), the RoboBrain LLM, the RoboOS Master, and the Gradio UI.

```bash
python3 remote_launch.py
```

A web browser window should open with the Gradio UI. If not, navigate to `http://127.0.0.1:7860`.

#### B. On the Slaver Robot Computer (`192.168.100.3`)

Run the local launch script. This will open new terminal tabs for Gazebo, RViz, and the RoboOS Slaver agent.

```bash
# Source your ROS2 environment first!
source /opt/ros/humble/setup.bash
python3 local_launch.py
```

#### C. Trigger the Task

1.  Go to the Gradio UI in your web browser.
2.  In the text box, type: `A patient at the Front Desk says they have a stomachache.`
3.  Click the "Submit Task" button.

## 5. Expected Outcome

1.  The Gradio UI will quickly show a "Success" message with the JSON plan from the Master.
2.  The Master's terminal will log the plan it received from RoboBrain and that it is dispatching the task.
3.  The Slaver's terminal will log that it has received the `guide_patient` subtask and is executing it.
4.  The Turtlebot3 robot in your Gazebo window will begin navigating from the "Front Desk" coordinates to the "Room 001" coordinates.
