#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import math

class MoveItCommanderNode(Node):
    def __init__(self):
        super().__init__('moveit_commander_node')
        
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.is_planning = False
        
        # --- RITA HARDWARE KINEMATICS ---
        self.MAX_REACH = 0.25 
        self.SAFETY_BUFFER = 0.07 
        self.DESIRED_REACH = self.MAX_REACH - self.SAFETY_BUFFER # 0.18m
        self.SHOULDER_HEIGHT = 0.11 
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Cartesian Commander Node ready. Anti-Flip constraints ACTIVE!")

    def timer_callback(self):
        if self.is_planning:
            return 
            
        try:
            # 1. Locate the face in the base frame
            trans = self.tf_buffer.lookup_transform('base', 'face_target', rclpy.time.Time())
            
            face_x = trans.transform.translation.x
            face_y = trans.transform.translation.y
            face_z = trans.transform.translation.z
            
            # 2. Vector from Shoulder to Face
            vec_x = face_x
            vec_y = face_y
            vec_z = face_z - self.SHOULDER_HEIGHT
            
            distance_to_face = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
            if distance_to_face < 0.01:
                return
                
            # 3. Scale to our safe reach distance
            if distance_to_face > self.DESIRED_REACH:
                scale = self.DESIRED_REACH / distance_to_face
            else:
                scale = 1.0 
                
            target_x = vec_x * scale
            target_y = vec_y * scale
            target_z = self.SHOULDER_HEIGHT + (vec_z * scale) 
            
            # Prevent hitting the floor
            target_z = max(0.08, target_z) 
            
            target_point = Point(x=target_x, y=target_y, z=target_z)
            self.send_moveit_goal(target_point)
            
        except Exception as e:
            pass

    def send_moveit_goal(self, target_point):
        self.get_logger().info(f"Targeting XYZ: [{target_point.x:.2f}, {target_point.y:.2f}, {target_point.z:.2f}]")
        self.is_planning = True
        
        self.action_client.wait_for_server()
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'rita_arm'
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 0.5
        
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # --- 1. POSITION CONSTRAINT ---
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base'
        pos_constraint.link_name = 'camera'
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.05] # 5cm tolerance for reach
        
        target_pose = Pose()
        target_pose.position = target_point
        target_pose.orientation.w = 1.0 
        
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0
        
        # --- 2. ANTI-FLIP ORIENTATION CONSTRAINT ---
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'base'
        ori_constraint.link_name = 'camera'
        
        # Base Upright Quaternion (Roll=0, Pitch=0, Yaw=0)
        ori_constraint.orientation.w = 1.0 
        
        # Tolerances (in radians)
        ori_constraint.absolute_z_axis_tolerance = 3.14 # YAW: Free to spin left/right completely
        ori_constraint.absolute_y_axis_tolerance = 1.57 # PITCH: Max 90 degrees up/down to stop backwards bending
        ori_constraint.absolute_x_axis_tolerance = 0.2  # ROLL: Strict 11-degree limit to prevent upside-down twisting
        
        ori_constraint.weight = 0.5
        
        # Package and send constraints
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the trajectory.")
            self.is_planning = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(f"MoveIt IK Failed (Error Code: {result.error_code.val}). Math too constrained.")
            
        self.is_planning = False

def main(args=None):
    rclpy.init(args=args)
    node = MoveItCommanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()