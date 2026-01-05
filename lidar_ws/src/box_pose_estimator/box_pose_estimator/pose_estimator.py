#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from vision_msgs.msg import Detection2DArray

class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        # 订阅 detectnet 的检测结果
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.detection_callback,
            10
        )

        # 相机内参矩阵 (你标定得到的)
        self.camera_matrix = np.array([
            [801.88276333,   0.0,          574.35783657],
            [0.0,            799.01656902, 351.34791443],
            [0.0,              0.0,           1.0]
        ], dtype=np.float32)

        # 畸变系数
        self.dist_coeffs = np.array([
            -0.360173928, 0.165948441, -0.000139283449,
            -0.0412332270, 0.0000597851194
        ], dtype=np.float32)

        # 方盒真实尺寸 (单位: 米)
        self.box_size = (0.26, 0.22, 0.20)  # 长x宽x高

    def detection_callback(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            return

        for det in msg.detections:
            # 提取 bounding box
            bbox = det.bbox
            cx, cy = bbox.center.position.x, bbox.center.position.y
            w, h = bbox.size_x, bbox.size_y

            # 计算四个角点 (像素坐标)
            image_points = np.array([
                [cx - w/2, cy - h/2],  # 左上
                [cx + w/2, cy - h/2],  # 右上
                [cx + w/2, cy + h/2],  # 右下
                [cx - w/2, cy + h/2],  # 左下
            ], dtype=np.float32)

            # 物体3D角点 (假设盒子中心在原点)
            L, W, H = self.box_size
            object_points = np.array([
                [-L/2, -W/2, 0],  # 左上
                [ L/2, -W/2, 0],  # 右上
                [ L/2,  W/2, 0],  # 右下
                [-L/2,  W/2, 0],  # 左下
            ], dtype=np.float32)

            # PnP 计算姿态
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE
            )

            if success:
                # 旋转向量转旋转矩阵
                R, _ = cv2.Rodrigues(rvec)
                self.get_logger().info(
                    f"Pose -> R:\n{R}\nT:{tvec.ravel()}"
                )

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
