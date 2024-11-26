#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  # 导入Float32消息类型
from cv_bridge import CvBridge


class ArucoAngleDetector:
    def __init__(self):
        rospy.init_node("aruco_angle_detector", anonymous=True)
        self.bridge = CvBridge()

        # 订阅相机图像话题
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback
        )

        # 发布角度到/dock_angle话题
        self.angle_pub = rospy.Publisher("/dock_angle", Float32, queue_size=10)

        # 设置Aruco字典和相机参数
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()

        # 配置相机内参矩阵和畸变系数
        self.camera_matrix = np.array(
            [[319.251271, 0, 315.526966], [0, 321.046284, 202.532319], [0, 0, 1]]
        )
        self.dist_coeffs = np.array(
            [0.006325, 0.017633, -0.004207, -0.000740, 0.000000]
        )

    def image_callback(self, data):
        # 将ROS图像消息转换为OpenCV格式
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # 转换为灰度图像
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 检测Aruco码
        corners, ids, _ = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters
        )

        if ids is not None:
            # 估计姿态
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, 0.27, self.camera_matrix, self.dist_coeffs
            )

            for i, rvec in enumerate(rvecs):
                # 获取相机到Aruco码的向量
                camera_to_marker = tvecs[i].flatten()

                # 创建该向量在YOZ平面的投影（忽略x分量）
                yoz_projection = np.array([0, camera_to_marker[1], camera_to_marker[2]])

                # 计算夹角
                angle_rad = np.arccos(
                    np.dot(camera_to_marker, yoz_projection)
                    / (
                        np.linalg.norm(camera_to_marker)
                        * np.linalg.norm(yoz_projection)
                    )
                )
                angle_deg = np.degrees(angle_rad)

                # 判断方向性，决定角度的正负
                if camera_to_marker[0] < 0:
                    angle_deg = -angle_deg

                # 打印角度信息
                rospy.loginfo(
                    f"Aruco Marker ID: {ids[i][0]}, YOZ Plane Angle: {angle_deg:.2f} degrees"
                )

                # 发布角度信息到/dock_angle话题
                self.angle_pub.publish(angle_deg)

                # 可视化Aruco码边框和轴
                aruco.drawDetectedMarkers(cv_image, corners)
                cv2.drawFrameAxes(
                    cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvecs[i], 0.1
                )

                # 在图像上绘制Aruco ID
                corner = corners[i][0]
                text_position = (int(corner[0][0]), int(corner[0][1]) - 10)
                cv2.putText(
                    cv_image,
                    f"ID: {ids[i][0]}",
                    text_position,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

        # 显示结果图像
        cv2.imshow("Aruco Angle Detector", cv_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    try:
        node = ArucoAngleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
