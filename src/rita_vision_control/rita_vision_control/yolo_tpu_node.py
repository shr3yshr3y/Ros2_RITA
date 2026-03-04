#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

try:
    from pycoral.utils.edgetpu import make_interpreter
    USE_PYCORAL = True
except ImportError:
    USE_PYCORAL = False
    try:
        from ai_edge_litert.interpreter import Interpreter, load_delegate
    except ImportError:
        from tensorflow.lite.python.interpreter import Interpreter, load_delegate

EDGETPU_SHARED_LIB = '/usr/lib/aarch64-linux-gnu/libedgetpu.so.1'


class YoloTpuNode(Node):
    def __init__(self):
        super().__init__('yolo_tpu_node')
        self.bridge = CvBridge()
        self.conf_threshold = 0.25
        self.nms_threshold = 0.45

        package_share = get_package_share_directory('rita_vision_control')
        tpu_model = os.path.join(package_share, 'models', 'best_int8_edgetpu.tflite')
        cpu_model = os.path.join(package_share, 'models', 'best_int8.tflite')

        self.interpreter = self._load_interpreter(tpu_model, cpu_model)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()[0]
        self.output_index = self.interpreter.get_output_details()[0]['index']
        self.input_h = int(self.input_details['shape'][1])
        self.input_w = int(self.input_details['shape'][2])

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
        )
        self.debug_pub = self.create_publisher(Image, '/camera/yolo_debug', 10)

        self.get_logger().info('YOLO face detector ready using models from camera_stuff.')

    def _load_interpreter(self, tpu_model, cpu_model):
        if not os.path.exists(tpu_model):
            raise FileNotFoundError(f'Model not found: {tpu_model}')

        if USE_PYCORAL:
            self.get_logger().info('Using pycoral EdgeTPU interpreter')
            return make_interpreter(tpu_model)

        try:
            delegate = load_delegate(EDGETPU_SHARED_LIB)
            self.get_logger().info('Using EdgeTPU delegate')
            return Interpreter(model_path=tpu_model, experimental_delegates=[delegate])
        except (ValueError, OSError, RuntimeError) as exc:
            self.get_logger().warn(f'EdgeTPU unavailable ({exc}), falling back to CPU model')
            if not os.path.exists(cpu_model):
                raise RuntimeError('No CPU fallback model found: best_int8.tflite') from exc
            return Interpreter(model_path=cpu_model)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_h, frame_w = frame.shape[:2]

            resized = cv2.resize(frame, (self.input_w, self.input_h))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            input_tensor = np.expand_dims(rgb, axis=0).astype(np.float32) / 255.0

            self.interpreter.set_tensor(self.input_details['index'], input_tensor)
            self.interpreter.invoke()

            output_data = self.interpreter.get_tensor(self.output_index)
            predictions = self._normalize_predictions(output_data)

            confidences = predictions[:, 4]
            mask = confidences > self.conf_threshold
            detections = predictions[mask]

            if len(detections) > 0:
                boxes_xywh = detections[:, :4].copy()
                boxes_for_nms = boxes_xywh.copy()
                boxes_for_nms[:, 0] -= boxes_for_nms[:, 2] / 2.0
                boxes_for_nms[:, 1] -= boxes_for_nms[:, 3] / 2.0

                boxes_abs = []
                for bx in boxes_for_nms:
                    x = int(bx[0] * frame_w)
                    y = int(bx[1] * frame_h)
                    w = int(bx[2] * frame_w)
                    h = int(bx[3] * frame_h)
                    boxes_abs.append([x, y, w, h])

                scores = detections[:, 4]
                indices = cv2.dnn.NMSBoxes(
                    boxes_abs,
                    scores.tolist(),
                    self.conf_threshold,
                    self.nms_threshold,
                )

                for idx in indices:
                    if isinstance(idx, (list, tuple, np.ndarray)):
                        idx = int(idx[0])
                    else:
                        idx = int(idx)

                    cx, cy, w, h = detections[idx, :4]
                    x1 = int((cx - w / 2.0) * frame_w)
                    y1 = int((cy - h / 2.0) * frame_h)
                    x2 = int((cx + w / 2.0) * frame_w)
                    y2 = int((cy + h / 2.0) * frame_h)

                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    radius = int(max(x2 - x1, y2 - y1) / 2)

                    cv2.circle(frame, (center_x, center_y), max(1, radius), (0, 255, 0), 2)
                    conf = float(detections[idx, 4])
                    cv2.putText(
                        frame,
                        f'{conf:.2f}',
                        (max(0, x1), max(0, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        1,
                    )
                    self.get_logger().info(
                        f'Face detected confidence={conf:.2f} center=({center_x},{center_y})'
                    )

            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        except Exception as exc:
            self.get_logger().error(f'Inference error: {exc}')

    @staticmethod
    def _normalize_predictions(output_data: np.ndarray) -> np.ndarray:
        arr = np.array(output_data)
        if arr.ndim == 3:
            arr = arr[0]

        if arr.ndim != 2:
            raise RuntimeError(f'Unexpected output shape: {output_data.shape}')

        if arr.shape[0] == 5:
            arr = arr.T

        if arr.shape[1] < 5:
            raise RuntimeError(f'Unexpected output layout after normalization: {arr.shape}')

        return arr[:, :5]


def main():
    rclpy.init()
    node = YoloTpuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
