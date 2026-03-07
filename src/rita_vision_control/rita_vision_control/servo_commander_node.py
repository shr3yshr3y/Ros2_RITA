#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

class ServoCommanderNode(Node):
    def __init__(self):
        super().__init__('servo_commander_node')
        
        self.get_logger().info("Connecting to MoveIt Servo...")
        self.cli = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/switch_command_type service...')
            
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.JOINT_JOG 
        future = self.cli.call_async(req)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_bounding_box',
            self.bbox_callback,
            10)
            
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        # --- CONTROL VARIABLES ---
        self.center_x = 960.0 
        self.center_y = 540.0 
        
        # Current commanded speeds
        self.pan_speed = 0.0
        self.tilt_speed = 0.0
        
        # Safety Tracking
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        
        # --- TUNING PARAMETERS ---
        self.gain_pan = 0.0015  # Lowered gain for smoother acceleration
        self.gain_tilt = 0.0015
        self.max_speed = 1.2   # Hard speed limit in radians/second
        self.deadzone = 40.0    # Stop moving if error is less than 40 pixels
        
        # Create a fast internal heartbeat (30Hz) to keep Servo awake!
        self.timer = self.create_timer(0.033, self.timer_callback)

    def bbox_callback(self, msg):
        cx, cy, w, h = msg.data
        
        # Record the exact time we saw a face
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        
        error_x = self.center_x - cx
        error_y = self.center_y - cy 
        
        # Apply Deadzone (If it's close to center, zero the error)
        if abs(error_x) < self.deadzone: error_x = 0.0
        if abs(error_y) < self.deadzone: error_y = 0.0
        
        # Calculate raw speed
        raw_pan = error_x * self.gain_pan
        raw_tilt = error_y * self.gain_tilt # Pitch up/down
        
        # Clamp speed to safe limits!
        self.pan_speed = max(min(raw_pan, self.max_speed), -self.max_speed)
        self.tilt_speed = max(min(raw_tilt, self.max_speed), -self.max_speed)

    def timer_callback(self):
        # Safety Stop: If YOLO hasn't seen a face in 0.5 seconds, slam the brakes
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_msg_time) > 0.5:
            self.pan_speed = 0.0
            self.tilt_speed = 0.0
            
        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()
        jog_msg.header.frame_id = 'base'
        jog_msg.joint_names = ['base_Revolute-1', 'link_2_Revolute-2', 'link_3_Revolute-3']
        
        # Constantly feed the current speed to MoveIt so it never stutters
        jog_msg.velocities = [
            float(self.pan_speed), 
            float(self.tilt_speed * 0.7), # Shoulder does 70% of the work
            float(self.tilt_speed * 0.3)  # Elbow does 30% of the work
        ]
        self.joint_pub.publish(jog_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ServoCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()