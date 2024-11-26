#!/usr/bin/env python3

import rospy
import csv
import matplotlib.pyplot as plt
from datetime import datetime


def plot_csv_data(csv_file):
    # 初始化存储数据的列表
    time = []
    current_yaw = []
    desired_yaw = []
    pid_throttle_output = []

    # 读取 CSV 文件
    try:
        with open(csv_file, mode="r") as file:
            reader = csv.DictReader(file)
            for row in reader:
                time.append(str(row["time"]))  # 保存原始时间戳
                current_yaw.append(float(row["current_yaw"]))
                desired_yaw.append(float(row["desired_yaw"]))
                pid_throttle_output.append(float(row["pid_throttle_output"]))
    except Exception as e:
        rospy.logerr(f"Failed to read CSV file: {e}")
        return

    # 转换时间格式为 HH:MM:SS
    time = [
        datetime.strptime(t, "%Y-%m-%d=%H:%M:%S.%f").strftime("%H:%M:%S") for t in time
    ]

    # 创建绘图
    fig, axs = plt.subplots(2, 1, figsize=(10, 10), sharex=True)

    # 第一张子图: Time vs Yaw Angles
    axs[0].plot(time, current_yaw, label="Current Yaw", color="blue")
    axs[0].plot(time, desired_yaw, label="Desired Yaw", color="red")
    axs[0].set_ylabel("Yaw Angle")
    axs[0].set_title("Time vs Current Yaw and Desired Yaw")
    axs[0].legend()
    axs[0].grid()

    # 第二张子图: Time vs pid_throttle_output
    axs[1].plot(time, pid_throttle_output, label="pid_throttle_output", color="green")
    axs[1].set_xlabel("Time (HH:MM:SS)")
    axs[1].set_ylabel("pid_throttle_output")
    axs[1].set_title("Time vs pid_throttle_output")
    axs[1].legend()
    axs[1].grid()

    # 调整横坐标刻度，避免重叠
    plt.xticks(rotation=45, ha="right")

    # 调整布局并显示
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("csv_plot_node")

    # CSV 文件路径 (修改为实际路径)
    csv_file_path = rospy.get_param(
        "~csv_file", "/home/shuai/ros_boat_ws/boat_data_pid_test.csv"
    )

    try:
        rospy.loginfo("Starting to plot CSV data...")
        plot_csv_data(csv_file_path)
        rospy.loginfo("Plotting finished.")
    except rospy.ROSInterruptException:
        pass
