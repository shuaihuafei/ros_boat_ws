#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import math


class PositionPublisher:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("position_publisher_node", anonymous=True)

        # 创建话题发布器
        self.local_pose_pub = rospy.Publisher(
            "/mavros/local_position/pose", PoseStamped, queue_size=10
        )
        self.global_position_pub = rospy.Publisher(
            "/mavros/global_position/global", NavSatFix, queue_size=10
        )

        # 发布频率
        self.rate = rospy.Rate(1)  # 每秒发布一次数据

        # 初始位置
        self.lon = 119.360371  # 经度
        self.lat = 32.115177  # 纬度
        self.alt = 0.0  # 高度

        # 初始局部坐标 (ENU 坐标系下)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.time_start = rospy.get_time()

    def publish_data(self):
        while not rospy.is_shutdown():
            current_time = rospy.get_time() - self.time_start  # 计算时间差

            # 更新局部坐标数据（模拟运动）
            self.x = current_time * 1.0
            self.y = current_time * 0.5
            self.z = current_time * 0.2
            self.yaw = current_time * 5  # 模拟旋转，角度每秒增加5度

            # 发布局部坐标
            local_pose = PoseStamped()
            local_pose.header.stamp = rospy.Time.now()
            local_pose.header.frame_id = "base_link"
            local_pose.pose.position.x = self.x
            local_pose.pose.position.y = self.y
            local_pose.pose.position.z = self.z

            # 将 yaw 转换为四元数
            quaternion = quaternion_from_euler(
                0, 0, math.radians(self.yaw)
            )  # 角度转弧度
            local_pose.pose.orientation.x = quaternion[0]
            local_pose.pose.orientation.y = quaternion[1]
            local_pose.pose.orientation.z = quaternion[2]
            local_pose.pose.orientation.w = quaternion[3]

            # 发布局部坐标数据
            self.local_pose_pub.publish(local_pose)

            # 更新全局坐标数据（经纬度）
            self.lon += 0.000011  # 模拟经度的变化
            self.lat += 0.000008  # 模拟纬度的变化

            global_position = NavSatFix()
            global_position.header.stamp = rospy.Time.now()
            global_position.header.frame_id = "gps"
            global_position.latitude = self.lat
            global_position.longitude = self.lon
            global_position.altitude = self.alt

            # 发布全局坐标数据
            self.global_position_pub.publish(global_position)

            # 设置发布频率
            self.rate.sleep()


if __name__ == "__main__":
    try:
        # 创建Publisher实例并开始发布数据
        position_publisher = PositionPublisher()
        position_publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
