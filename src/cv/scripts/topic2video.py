#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# 视频文件的保存路径
output_video = "/home/shuai/Videos/output_video.mp4"

# 初始化视频写入
fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # 使用 mp4v 编码
video_writer = None


# ROS 节点回调函数
def image_callback(msg):
    global video_writer

    # 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 格式的图像
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))
        return

    # 初始化视频写入器（如果还没有初始化）
    if video_writer is None:
        height, width, _ = cv_image.shape
        video_writer = cv2.VideoWriter(output_video, fourcc, 20.0, (width, height))

    # 将当前帧写入视频文件
    video_writer.write(cv_image)
    rospy.loginfo("Frame written to video")


def main():
    # 初始化 ROS 节点
    rospy.init_node("image_to_video", anonymous=True)

    # 订阅话题 /yolo/predict_dock
    rospy.Subscriber("/yolo/predict_dock", Image, image_callback)

    rospy.loginfo("ROS Node started, converting images to video.")

    # 保持节点持续运行
    rospy.spin()

    # 程序结束时，释放视频写入器资源
    if video_writer is not None:
        video_writer.release()
        rospy.loginfo("Video file saved at: %s", output_video)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
