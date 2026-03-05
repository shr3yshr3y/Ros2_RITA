#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf2_ros

class FaceTFBroadcaster(Node):
    def __init__(self):
        super().__init__('face_tf_broadcaster')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_bounding_box',
            self.bbox_callback,
            10)
            
        # Initialize the transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Camera constants for 1920x1080 resolution
        self.focal_length = 1145.0  # Approximated focal length for a standard 1080p webcam FOV
        self.real_face_width = 0.15 # 15cm average human face width
        
        self.get_logger().info("Face TF Broadcaster started. Open RViz to see the 'face_target' frame!")

    def bbox_callback(self, msg):
        cx, cy, w, h = msg.data
        
        # 1. Calculate depth (Z-distance from camera)
        depth = (self.real_face_width * self.focal_length) / w
        
        # 2. Use the TRUE center of your 1080p camera image
        image_center_x = 960.0 
        image_center_y = 540.0 
        
        offset_x = (cx - image_center_x) * depth / self.focal_length
        offset_y = (cy - image_center_y) * depth / self.focal_length
        
        # 3. Create the Transform Message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera'
        t.child_frame_id = 'face_target'
        
        # Optical Frame Mapping (Blue Z is forward)
        t.transform.translation.x = float(offset_x)  
        t.transform.translation.y = float(offset_y)  
        t.transform.translation.z = float(depth)
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FaceTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()