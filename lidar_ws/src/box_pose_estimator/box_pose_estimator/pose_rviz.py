#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import cv2
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PoseRViz(Node):
    def __init__(self):
        super().__init__('pose_RViz')

        # 订阅 detectnet 的检测结果
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detectnet/detections',
            self.listener_callback,
            10)

        # 发布 TF
        self.tf_broadcaster = TransformBroadcaster(self)

        # ====== 相机标定参数 (你的标定结果) ======
        self.camera_matrix = np.array([
            [801.88276333,   0.0,          574.35783657],
            [0.0,            799.01656902, 351.34791443],
            [0.0,              0.0,           1.0]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            -0.360173928,
             0.165948441,
            -0.000139283449,
            -0.0412332270,
             0.0000597851194
        ], dtype=np.float32)

        # ====== 方盒真实尺寸 (米) ======
        self.box_size = (0.26, 0.22, 0.20)  # 长、宽、高

        # 定义物体的3D点 (假设盒子中心在原点)
        L, W, H = self.box_size
        self.obj_points = np.array([
            [-L/2, -W/2, 0],  # 左上
            [ L/2, -W/2, 0],  # 右上
            [ L/2,  W/2, 0],  # 右下
            [-L/2,  W/2, 0],  # 左下
        ], dtype=np.float32)

    def listener_callback(self, msg: Detection2DArray):
        if len(msg.detections) == 0:
            return

        det = msg.detections[0]
        bbox = det.bbox
        cx, cy = bbox.center.position.x, bbox.center.position.y
        w, h = bbox.size_x, bbox.size_y

        # 2D 像素点 (左上、右上、右下、左下)
        img_points = np.array([
            [cx - w/2, cy - h/2],
            [cx + w/2, cy - h/2],
            [cx + w/2, cy + h/2],
            [cx - w/2, cy + h/2],
        ], dtype=np.float32)

        # 使用 solvePnP 计算位姿
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            img_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not success:
            self.get_logger().warn("solvePnP failed")
            return

        # 构造 TransformStamped
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = "box"

        transform.transform.translation.x = float(tvec[0])
        transform.transform.translation.y = float(tvec[1])
        transform.transform.translation.z = float(tvec[2])

        # Rodrigues -> 四元数
        R, _ = cv2.Rodrigues(rvec)
        qw = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)

        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # 发布 TF
        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(
            f"Published TF: t=({tvec[0][0]:.2f}, {tvec[1][0]:.2f}, {tvec[2][0]:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseRViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

