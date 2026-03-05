#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
import tf2_ros
import tf2_geometry_msgs
import math

class MoveItCommanderNode(Node):
    def __init__(self):
        super().__init__('moveit_commander_node')
        
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_coords_xyz',
            self.coords_callback,
            1)
            
        self.is_planning = False
        
        # --- RITA HARDWARE LIMITS ---
        # Set this to slightly less than your arm's fully extended length (in meters)
        self.MAX_REACH = 0.25 # 25 centimeters
        
        self.get_logger().info("MoveIt 2 Commander Node ready. Waiting for XYZ coordinates...")

    def coords_callback(self, msg):
        if self.is_planning:
            return 
            
        cam_x, cam_y, cam_z = msg.data
        
        # 1. Map the face in the camera's local 3D space
        target_pt_cam = PointStamped()
        target_pt_cam.header.frame_id = 'camera'
        target_pt_cam.header.stamp = self.get_clock().now().to_msg()
        target_pt_cam.point.x = cam_x
        target_pt_cam.point.y = cam_y
        target_pt_cam.point.z = cam_z
        
        try:
            # 2. Transform the face's location into the global 'base' frame
            transform = self.tf_buffer.lookup_transform('base', 'camera', rclpy.time.Time())
            target_pt_base = tf2_geometry_msgs.do_transform_point(target_pt_cam, transform)
            
            face_x = target_pt_base.point.x
            face_y = target_pt_base.point.y
            face_z = target_pt_base.point.z
            
            # 3. Calculate absolute distance from robot base to the human face
            distance_to_face = math.sqrt(face_x**2 + face_y**2 + face_z**2)
            
            if distance_to_face < 0.01:
                return
                
            # 4. CRITICAL MATH: Keep it reachable!
            # Scale the target down so it sits perfectly inside the robot's physical reach
            if distance_to_face > self.MAX_REACH:
                scale = self.MAX_REACH / distance_to_face
            else:
                scale = 1.0 # The face is close enough to reach normally
                
            reachable_x = face_x * scale
            reachable_y = face_y * scale
            
            # Ensure the arm doesn't smash into the table/floor trying to look down
            reachable_z = max(0.1, face_z * scale) 
            
            reachable_target = PointStamped()
            reachable_target.point.x = reachable_x
            reachable_target.point.y = reachable_y
            reachable_target.point.z = reachable_z
            
            self.send_moveit_goal(reachable_target.point)
            
        except Exception as e:
            self.get_logger().warn(f"Waiting for TF Tree: {e}")

    def send_moveit_goal(self, target_point):
        self.get_logger().info(f"Target Acquired. Planning to REACHABLE Base XYZ: [{target_point.x:.2f}, {target_point.y:.2f}, {target_point.z:.2f}]")
        self.is_planning = True
        
        self.action_client.wait_for_server()
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'rita_arm'
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.allowed_planning_time = 0.5
        
        # Keep speed moderate for tracking
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'base'
        pos_constraint.link_name = 'camera'
        
        # Tolerance sphere (8cm) to give the 3-DOF IK solver some breathing room
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.08] 
        
        target_pose = Pose()
        target_pose.position = target_point
        target_pose.orientation.w = 1.0
        
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(target_pose)
        pos_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        goal_msg.request.goal_constraints.append(constraints)
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the target (Out of reach/Collision).")
            self.is_planning = False
            return

        self.get_logger().info("MoveIt executing trajectory...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        # MoveIt Error Code 1 means SUCCESS!
        if result.error_code.val == 1:
            self.get_logger().info("Move complete. Ready for next frame.")
        else:
            self.get_logger().error(f"MoveIt failed (Error Code: {result.error_code.val}). KDL could not find a joint angle for this XYZ.")
            
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