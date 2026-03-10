#!/usr/bin/env python3

import os
# Force TCP for RTSP to prevent corruption
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class YoloWebRTCNode(Node):
    def __init__(self):
        super().__init__('yolo_webrtc_node')
        
        self.conf_threshold = 0.50

        # --- LOAD YOLO MODEL FOR NVIDIA GPU ---
        # If you have your original best.pt, put it in the models folder and change the name here.
        # Otherwise, YOLO will automatically download 'yolov8n.pt' to your folder on first run.
        package_share = get_package_share_directory('rita_vision_control')
        model_path = os.path.join(package_share, 'models', 'best.pt') 
        
        self.get_logger().info(f"Loading native YOLO model onto RTX 2070...")
        
        # This single line automatically detects your RTX 2070 and loads it onto the VRAM!
        self.model = YOLO(model_path) 
        
        # Publisher for the bounding box
        self.bbox_pub = self.create_publisher(Float32MultiArray, '/target_bounding_box', 10)
        
        # RTSP Stream URL from Pi
        self.stream_url = "rtsp://100.124.114.11:8554/cam"
        
        self.thread = threading.Thread(target=self.video_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f"YOLO GPU Node started. Pulling from {self.stream_url} via TCP")

    def video_loop(self):
        cap = cv2.VideoCapture(self.stream_url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                continue

            try:
                frame_h, frame_w = frame.shape[:2]

                # --- NATIVE GPU INFERENCE ---
                # We tell YOLO to run silently (verbose=False) to prevent terminal spam
                results = self.model(frame, verbose=False, conf=self.conf_threshold)

                # Get the first result (since we only passed one frame)
                result = results[0]
                
                # Check if anything was detected
                if len(result.boxes) > 0:
                    # Get the most confident detection's bounding box in [x_center, y_center, width, height] format
                    box = result.boxes[0].xywh[0] 
                    conf = result.boxes[0].conf[0]
                    cls_id = int(result.boxes[0].cls[0])
                    
                    # Example: If you want it to ONLY track 'person' (Class 0 in standard YOLO)
                    if cls_id == 0:
                        cx, cy, w, h = float(box[0]), float(box[1]), float(box[2]), float(box[3])

                        # Draw targeting circle on the frame
                        radius = int(max(w, h) / 2)
                        cv2.circle(frame, (int(cx), int(cy)), max(1, radius), (0, 255, 0), 2)
                        cv2.putText(frame, f'Person {conf:.2f}', (int(cx - w/2), int(cy - h/2 - 5)), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Publish to Servo Commander
                        bbox_msg = Float32MultiArray()
                        bbox_msg.data = [cx, cy, w, h]
                        self.bbox_pub.publish(bbox_msg)
                    
            except Exception as exc:
                self.get_logger().error(f'Inference error: {exc}')

            # Display the 1080p 60fps feed
            cv2.imshow("RITA Eye - RTX 2070 GPU Inference", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def main():
    rclpy.init()
    node = YoloWebRTCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()