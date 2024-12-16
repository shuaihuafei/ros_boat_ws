#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def video_to_ros():
    # 初始化 ROS 节点
    rospy.init_node("video_to_ros_node", anonymous=True)

    # 创建一个图像发布器
    image_pub = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)

    # 创建 CvBridge 对象
    bridge = CvBridge()

    # 打开视频文件
    video_path = "/home/shuai/Videos/无人艇回坞-第一视角.mp4"
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        rospy.logerr("无法打开视频文件")
        return

    # 获取视频的帧率
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    # 设置循环发布图像
    rate = rospy.Rate(fps)  # 设置发布频率与视频帧率一致

    while not rospy.is_shutdown():
        # 读取一帧图像
        ret, frame = cap.read()

        if not ret:
            rospy.loginfo("视频读取完毕")
            break

        try:
            # 将 OpenCV 图像转换为 ROS 图像消息
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # 发布图像
            image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")

        # 按照帧率发布图像
        rate.sleep()

    # 释放视频资源
    cap.release()


if __name__ == "__main__":
    try:
        video_to_ros()
    except rospy.ROSInterruptException:
        pass
