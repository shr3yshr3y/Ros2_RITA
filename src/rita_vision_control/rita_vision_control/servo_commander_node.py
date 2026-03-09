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
        
        # --- CONTROL VARIABLES (Hardcoded to 640x480) ---
        self.image_width = 640.0
        self.image_height = 480.0
        self.center_x = 320.0 
        self.center_y = 240.0 
        
        self.pan_speed = 0.0
        self.depth_speed = 0.0
        self.tilt_speed = 0.0
        
        self.last_msg_time = self.get_clock().now().nanoseconds / 1e9
        self.last_log_time = 0.0  # Used to throttle debug messages
        
        # Current Joint Positions
        self.current_joint_2_pos = 0.0
        self.current_joint_3_pos = 0.0
        
        # --- PID STATE VARIABLES (XY Only) ---
        self.integral_x = 0.0
        self.prev_error_x = 0.0
        
        self.integral_y = 0.0
        self.prev_error_y = 0.0
        
        # --- TUNING & LIMITS PARAMETERS ---
        self.kp_pan = 0.003
        self.ki_pan = 0.0001
        self.kd_pan = 0.002 
        
        self.kp_tilt = 0.003
        self.ki_tilt = 0.0001
        self.kd_tilt = 0.002
        
        # Joint 2 Limits (Distance Tracking)
        self.j2_max_pos = 1.57   
        self.j2_max_neg = -1.57  
        self.position_kp = 1.5   
        
        # Joint 3 Limits (Tilt Takeover Safety)
        self.j3_max_pos = 1.57   
        self.j3_max_neg = -1.57  
        self.j3_limit_tolerance = 0.05 
        
        # 5 degrees in radians
        self.j3_freeze_threshold = 5.0 * (math.pi / 180.0) 
        
        self.max_speed = 3      
        # Dynamically set deadzone to ~2% of image width (640 * 0.02 = ~12.8 pixels)
        self.deadzone_xy = self.image_width * 0.02
        self.max_integral = 500.0 

        # Fast internal heartbeat (30Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.get_logger().info("Servo Commander Initialized (Hardcoded 640x480)")

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
        w_min = self.image_width / 20.0  
        w_max = self.image_width / 3.0  
        w_clamped = max(min(w, w_max), w_min)
        if (w_max - w_min) == 0:
            ratio = 0.0  # Fallback to prevent crash
        else:
            ratio = (w_clamped - w_min) / (w_max - w_min)
        
        target_angle = self.j2_max_neg + (ratio * (self.j2_max_pos - self.j2_max_neg))
        position_error = target_angle - self.current_joint_2_pos
        
        # Calculate the intended depth velocity before takeover overrides it
        if abs(position_error) < 0.05:
            intended_raw_depth = 0.0
        else:
            intended_raw_depth = position_error * self.position_kp
            
        # --- JOINT 3 TAKEOVER LOGIC ---
        at_max_limit = self.current_joint_3_pos >= (self.j3_max_pos - self.j3_limit_tolerance)
        at_min_limit = self.current_joint_3_pos <= (self.j3_max_neg + self.j3_limit_tolerance)
        
        # Check if J3 is at the limit AND the PID wants it to push further into that limit
        pegged_at_max = at_max_limit and (raw_tilt > 0.0)
        pegged_at_min = at_min_limit and (raw_tilt < 0.0)
        
        # Check if J3 is within freeze threshold of either limit
        near_max_limit = self.current_joint_3_pos >= (self.j3_max_pos - self.j3_freeze_threshold)
        near_min_limit = self.current_joint_3_pos <= (self.j3_max_neg + self.j3_freeze_threshold)
        
        status_msg = "Normal Tracking"
        
        if pegged_at_max or pegged_at_min:
            # 1. Compensating: J3 is pegged. J2 takes over tilt velocity.
            raw_depth = raw_tilt
            raw_tilt = 0.0  
            status_msg = "COMPENSATING: J3 locked, J2 taking over tilt"
        elif near_max_limit or near_min_limit:
            # 2. Frozen check: Compare velocity signs to unlock if they share a direction
            # Using multiplication: a positive product means they have the same sign (same direction)
            if intended_raw_depth * raw_tilt > 0.0:
                raw_depth = intended_raw_depth
                status_msg = "UNLOCKED: J2 and J3 share velocity direction"
            else:
                raw_depth = 0.0
                status_msg = "FROZEN: J3 near limit, J2 halted"
        else:
            # 3. Normal tracking
            raw_depth = intended_raw_depth
            
        # --- ENSURE J3 TRACKS FASTER THAN J2 ---
        # If both joints are moving normally (not in a compensation state)
        if status_msg != "COMPENSATING: J3 locked, J2 taking over tilt":
            if abs(raw_tilt) > 0.01 and abs(raw_depth) > 0.01:
                # If Joint 2 is trying to move faster than Joint 3
                if abs(raw_depth) >= abs(raw_tilt):
                    # Scale Joint 2's speed down to 85% of Joint 3's speed (preserving J2's direction)
                    raw_depth = math.copysign(abs(raw_tilt) * 0.85, raw_depth)
                    status_msg += " (J2 speed capped by J3)"
        
        # --- DEBUG LOGGING ---
        # Print a debug message every 0.5 seconds to avoid spamming the terminal
        if current_time - self.last_log_time > 0.5:
            scale_percent = ratio * 100.0
            self.get_logger().info(
                f"\n--- DEBUG INFO ---\n"
                f"Face Scale: {scale_percent:.1f}%\n"
                f"J2 Target Angle: {target_angle:.2f} rad\n"
                f"J2 Current Angle: {self.current_joint_2_pos:.2f} rad\n"
                f"Status: {status_msg}\n"
                f"------------------"
            )
            self.last_log_time = current_time
        
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