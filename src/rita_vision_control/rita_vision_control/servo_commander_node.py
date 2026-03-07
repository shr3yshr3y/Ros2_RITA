#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from sensor_msgs.msg import JointState
import math

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
        
        # --- SUBSCRIBERS & PUBLISHERS ---
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_bounding_box',
            self.bbox_callback,
            10)
            
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
            
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        # --- CONTROL VARIABLES ---
        self.image_width = 1920.0
        self.center_x = 960.0 
        self.center_y = 540.0 
        
        self.pan_speed = 0.0
        self.depth_speed = 0.0
        self.tilt_speed = 0.0
        
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        
        # Current Joint Positions
        self.current_joint_2_pos = 0.0
        self.current_joint_3_pos = 0.0
        
        # --- PID STATE VARIABLES (XY Only) ---
        self.integral_x = 0.0
        self.prev_error_x = 0.0
        
        self.integral_y = 0.0
        self.prev_error_y = 0.0
        
        # --- TUNING & LIMITS PARAMETERS ---
        self.kp_pan = 0.0015
        self.ki_pan = 0.0001
        self.kd_pan = 0.0005 
        
        self.kp_tilt = 0.0015
        self.ki_tilt = 0.0001
        self.kd_tilt = 0.0005
        
        # Joint 2 Limits (Distance Tracking)
        self.j2_max_pos = 1.57   
        self.j2_max_neg = -1.57  
        self.position_kp = 1.5   
        
        # Joint 3 Limits (Tilt Takeover Safety)
        self.j3_max_pos = 1.57   
        self.j3_max_neg = -1.57  
        self.j3_limit_tolerance = 0.05 
        
        # 15 degrees in radians (~0.2618 rad)
        self.j3_freeze_threshold = 15.0 * (math.pi / 180.0) 
        
        self.max_speed = 1.2      
        self.deadzone_xy = 40.0   
        self.max_integral = 500.0 

        # Fast internal heartbeat (30Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)

    def joint_state_callback(self, msg):
        try:
            # Update current positions of Joint 2 and Joint 3
            if 'link_2_Revolute-2' in msg.name:
                index_j2 = msg.name.index('link_2_Revolute-2')
                self.current_joint_2_pos = msg.position[index_j2]
                
            if 'link_3_Revolute-3' in msg.name:
                index_j3 = msg.name.index('link_3_Revolute-3')
                self.current_joint_3_pos = msg.position[index_j3]
        except ValueError:
            pass

    def bbox_callback(self, msg):
        cx, cy, w, h = msg.data
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_msg_time
        if dt <= 0.0 or dt > 1.0:
            dt = 0.033 
        self.last_msg_time = current_time
        
        # --- PAN & TILT PID CONTROL ---
        error_x = self.center_x - cx
        error_y = self.center_y - cy 
        
        if abs(error_x) < self.deadzone_xy: 
            error_x = 0.0
            self.integral_x = 0.0  
        if abs(error_y) < self.deadzone_xy: 
            error_y = 0.0
            self.integral_y = 0.0  
            
        self.integral_x += error_x * dt
        self.integral_x = max(min(self.integral_x, self.max_integral), -self.max_integral)
        derivative_x = (error_x - self.prev_error_x) / dt
        raw_pan = (self.kp_pan * error_x) + (self.ki_pan * self.integral_x) + (self.kd_pan * derivative_x)
        self.prev_error_x = error_x
        
        self.integral_y += error_y * dt
        self.integral_y = max(min(self.integral_y, self.max_integral), -self.max_integral)
        derivative_y = (error_y - self.prev_error_y) / dt
        raw_tilt = (self.kp_tilt * error_y) + (self.ki_tilt * self.integral_y) + (self.kd_tilt * derivative_y)
        self.prev_error_y = error_y
        
        # --- DEPTH CONTROL (Linear Scale for Joint 2) ---
        w_min = self.image_width / 9.0  
        w_max = self.image_width / 2.0  
        w_clamped = max(min(w, w_max), w_min)
        ratio = (w_clamped - w_min) / (w_max - w_min)
        target_angle = self.j2_max_pos + (ratio * (self.j2_max_pos - self.j2_max_neg))
        position_error = target_angle - self.current_joint_2_pos
            
        # --- JOINT 3 TAKEOVER LOGIC ---
        at_max_limit = self.current_joint_3_pos >= (self.j3_max_pos - self.j3_limit_tolerance)
        at_min_limit = self.current_joint_3_pos <= (self.j3_max_neg + self.j3_limit_tolerance)
        
        # Check if J3 is at the limit AND the PID wants it to push further into that limit
        pegged_at_max = at_max_limit and (raw_tilt > 0.0)
        pegged_at_min = at_min_limit and (raw_tilt < 0.0)
        
        # Check if J3 is within 15 degrees of either limit
        near_max_limit = self.current_joint_3_pos >= (self.j3_max_pos - self.j3_freeze_threshold)
        near_min_limit = self.current_joint_3_pos <= (self.j3_max_neg + self.j3_freeze_threshold)
        
        if pegged_at_max or pegged_at_min:
            # 1. Compensating: J3 is pegged. J2 takes over tilt velocity.
            # NOTE: If J2 pitches the arm in the *opposite* direction of J3, change this to `-raw_tilt`
            raw_depth = raw_tilt
            raw_tilt = 0.0  # Stop sending commands to the pegged joint
        elif near_max_limit or near_min_limit:
            # 2. Frozen: J3 no longer needs to push further, but is within 15 degrees of the limit.
            raw_depth = 0.0
        else:
            # 3. Normal: J3 is far from limits, J2 performs normal distance tracking.
            if abs(position_error) < 0.05:
                raw_depth = 0.0
            else:
                raw_depth = position_error * self.position_kp
        
        # --- CLAMP SPEEDS ---
        self.pan_speed = max(min(raw_pan, self.max_speed), -self.max_speed)
        self.depth_speed = max(min(raw_depth, self.max_speed), -self.max_speed)
        self.tilt_speed = max(min(raw_tilt, self.max_speed), -self.max_speed)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        time_since_last_msg = current_time - self.last_msg_time
        
        if time_since_last_msg > 0.2:
            self.pan_speed = 0.0
            self.depth_speed = 0.0
            self.tilt_speed = 0.0
            self.integral_x = 0.0
            self.integral_y = 0.0
        else:
            if time_since_last_msg > 0.06:
                decay_factor = 0.85 
                self.pan_speed *= decay_factor
                self.depth_speed *= decay_factor
                self.tilt_speed *= decay_factor
            
        jog_msg = JointJog()
        jog_msg.header.stamp = self.get_clock().now().to_msg()
        jog_msg.header.frame_id = 'base'
        jog_msg.joint_names = ['base_Revolute-1', 'link_2_Revolute-2', 'link_3_Revolute-3']
        
        jog_msg.velocities = [
            float(self.pan_speed), 
            float(self.depth_speed), 
            float(self.tilt_speed)
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