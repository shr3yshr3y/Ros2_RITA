#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')
        
        # Subscribes to the YOLO output from your TPU node
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_bounding_box',
            self.servo_callback,
            10)
            
        # Publisher for the calculated Cartesian coordinates
        self.publisher = self.create_publisher(Float32MultiArray, '/target_coords_xyz', 10)
        
        # Camera constants for 320x320 resolution
        self.focal_length = 200.0  # Approximation
        self.real_face_width = 0.15 # 15cm
        
        self.get_logger().info("Visual Servo Node started. Tracking XYZ...")

    def servo_callback(self, msg):
        # Coordinates from YOLO: [center_x, center_y, width, height]
        cx, cy, w, h = msg.data
        
        # Calculate Z (Depth) based on face size
        # Smaller face = further away, bigger face = closer
        z = (self.real_face_width * self.focal_length) / w
        
        # Calculate X and Y offsets from center
        x = (cx - 160) * z / self.focal_length
        y = (cy - 160) * z / self.focal_length
        
        # Publish the XYZ target
        out_msg = Float32MultiArray()
        out_msg.data = [float(x), float(y), float(z)]
        self.publisher.publish(out_msg)

# This is the 'main' attribute ROS was looking for!
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