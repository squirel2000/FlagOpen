import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from flask import Flask, request, jsonify
import threading
import json

# --- ROS2 Action Node ---
class MCP_ROS2_Bridge(Node):
    def __init__(self):
        super().__init__('mcp_ros2_bridge')
        self.get_logger().info('MCP-ROS2 Bridge starting...')
        # Create an action client for the navigate_to_pose action
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server found.')

    def send_navigation_goal(self, x, y, z, w):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = z
        goal_msg.pose.pose.orientation.w = w

        self.get_logger().info('Sending navigation goal...')
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
        # This is a simplified, non-blocking call. A real implementation
        # would monitor the future for completion and handle results/errors.
        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: Distance remaining: {feedback.distance_remaining}')

# --- Flask Server for MCP ---
app = Flask(__name__)
ros_node = None

# This is our "database" of room coordinates. In a real system, this might come from a map server.
ROOM_COORDINATES = {
    "001": {"x": 1.5, "y": -2.5, "z": 0.0, "w": 1.0},
    "102": {"x": 5.0, "y": 3.0, "z": 0.0, "w": 1.0},
}

@app.route('/execute_tool', methods=['POST'])
def execute_tool():
    global ros_node
    data = request.get_json()
    tool_name = data.get('tool_name')
    args = data.get('args', {})
    
    if tool_name == 'guide_patient_to_room':
        room_number = args.get('room_number')
        if room_number in ROOM_COORDINATES:
            coords = ROOM_COORDINATES[room_number]
            # In a real scenario, this might be a sequence: navigate -> speak
            # Here, we just trigger the navigation.
            success = ros_node.send_navigation_goal(coords['x'], coords['y'], coords['z'], coords['w'])
            if success:
                return jsonify({"status": "SUCCESS", "message": f"Navigation goal for room {room_number} sent."})
            else:
                return jsonify({"status": "FAILURE", "message": "Failed to send navigation goal."}), 500
        else:
            return jsonify({"status": "FAILURE", "message": f"Unknown room number: {room_number}"}), 404
            
    elif tool_name == 'speak':
        # Here you would call a ROS2 service or topic for TTS
        message = args.get('message', 'No message provided.')
        ros_node.get_logger().info(f"MCP Server received speak command: '{message}'")
        return jsonify({"status": "SUCCESS", "message": "Speak command received."})
        
    else:
        return jsonify({"status": "FAILURE", "message": f"Unknown tool: {tool_name}"}), 400

def run_flask():
    # Running on port 8081 to avoid conflict with RoboBrain server (8001)
    app.run(host='0.0.0.0', port=8081)

def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = MCP_ROS2_Bridge()
    
    # Run Flask in a separate thread
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    
    rclpy.spin(ros_node)
    
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
