#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_bounding_box',
            self.servo_callback,
            10)
            
        self.publisher = self.create_publisher(Float32MultiArray, '/target_coords_xyz', 10)
        
        # --- TF2 Setup: To track the position and pointing direction of the arm ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Camera constants for 320x320 resolution
        self.focal_length = 200.0  
        self.real_face_width = 0.15 # 15cm
        
        self.get_logger().info("Visual Servo Node started. Computing TRUE GLOBAL XYZ...")

    def servo_callback(self, msg):
        cx, cy, w, h = msg.data
        
        # 1. Calculate Local Camera Coordinates
        depth = (self.real_face_width * self.focal_length) / w
        offset_x = (cx - 160) * depth / self.focal_length
        offset_y = (cy - 160) * depth / self.focal_length
        
        # 2. Create a Point in the Camera's Local Frame
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera' 
        point_cam.header.stamp = self.get_clock().now().to_msg()
        
        # NOTE: Standard ROS camera optical frames point Z forward, X right, Y down.
        # If your 'camera' URDF link points X forward instead (like a standard mechanical link),
        # you will need to swap these to: x = depth, y = -offset_x, z = -offset_y
        point_cam.point.x = offset_x
        point_cam.point.y = offset_y
        point_cam.point.z = depth
        
        # 3. Apply the TF2 Math (This handles End Effector Position AND Pointing Direction)
        try:
            # Get the exact matrix bridging the 'camera' to the global 'base'
            transform = self.tf_buffer.lookup_transform('base', 'camera', rclpy.time.Time())
            
            # Project the local point into the global base frame
            point_base = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            
            true_x = point_base.point.x
            true_y = point_base.point.y
            true_z = point_base.point.z
            
            # Publish the TRUE Global XYZ target
            out_msg = Float32MultiArray()
            out_msg.data = [float(true_x), float(true_y), float(true_z)]
            self.publisher.publish(out_msg)
            
            # Print it so you can see the math working!
            self.get_logger().info(f"Target at True Base XYZ: [{true_x:.2f}, {true_y:.2f}, {true_z:.2f}]")
            
        except Exception as e:
            self.get_logger().warn(f"Waiting for arm positional data (TF Tree): {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()