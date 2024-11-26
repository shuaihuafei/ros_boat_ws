#!/usr/bin/env python3

import rospy
import csv
from pyecharts import options as opts
from pyecharts.charts import Line
from datetime import datetime
import os


def plot_csv_data(csv_file, output_path):
    # 初始化存储数据的列表
    time = []
    current_yaw = []
    desired_yaw = []
    pid_throttle_output = []

    # 拼接 CSV 文件路径
    csv_file_path = os.path.join(output_path, csv_file)

    # 读取 CSV 文件
    try:
        with open(csv_file_path, mode="r") as file:
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

    # 动态计算 Y 轴范围
    def calculate_y_range(data):
        min_val = min(data)
        max_val = max(data)
        padding = (max_val - min_val) * 0.1  # 10% 的填充量
        return min_val - padding, max_val + padding

    # 获取 "current_yaw" 和 "desired_yaw" 的 Y 轴范围
    min_yaw, max_yaw = calculate_y_range(current_yaw + desired_yaw)
    # 获取 "pid_throttle_output" 的 Y 轴范围
    min_throttle, max_throttle = calculate_y_range(pid_throttle_output)

    # 创建第一张图：Time vs Yaw Angles
    line1 = (
        Line()
        .add_xaxis(time)  # 添加横坐标数据
        .add_yaxis("Current Yaw", current_yaw, color="blue")  # 添加 "Current Yaw" 曲线
        .add_yaxis("Desired Yaw", desired_yaw, color="red")  # 添加 "Desired Yaw" 曲线
        .set_global_opts(
            title_opts=opts.TitleOpts(
                title="Time vs Yaw Angles", subtitle="Current Yaw vs Desired Yaw"
            ),
            xaxis_opts=opts.AxisOpts(
                name="Time (HH:MM:SS)", axislabel_opts=opts.LabelOpts(rotate=45)
            ),
            yaxis_opts=opts.AxisOpts(
                name="Yaw Angle", min_=round(min_yaw, 2), max_=round(max_yaw, 2)
            ),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            tooltip_opts=opts.TooltipOpts(trigger="axis", axis_pointer_type="cross"),
        )
    )

    # 创建第二张图：Time vs PID Throttle Output
    line2 = (
        Line()
        .add_xaxis(time)  # 添加横坐标数据
        .add_yaxis(
            "PID Throttle Output", pid_throttle_output, color="green"
        )  # 添加 "PID Throttle Output" 曲线
        .set_global_opts(
            title_opts=opts.TitleOpts(title="Time vs PID Throttle Output"),
            xaxis_opts=opts.AxisOpts(
                name="Time (HH:MM:SS)", axislabel_opts=opts.LabelOpts(rotate=45)
            ),
            yaxis_opts=opts.AxisOpts(
                name="PID Throttle Output",
                min_=round(min_throttle, 2),
                max_=round(max_throttle, 2),
            ),
            toolbox_opts=opts.ToolboxOpts(is_show=True),
            tooltip_opts=opts.TooltipOpts(trigger="axis", axis_pointer_type="cross"),
        )
    )

    # 渲染图表到单独的 HTML 文件
    yaw_angles_chart_path = os.path.join(output_path, "01yaw_angles_chart.html")
    pid_throttle_output_chart_path = os.path.join(
        output_path, "02pid_throttle_output_chart.html"
    )

    line1.render(yaw_angles_chart_path)
    line2.render(pid_throttle_output_chart_path)

    # 创建一个 HTML 文件将两个图表通过 iframe 嵌入其中
    with open(os.path.join(output_path, "00combined_chart.html"), "w") as f:
        f.write(
            """
            <html>
                <head>
                    <title>Combined Charts</title>
                </head>
                <body>
                    <h1>Time vs Yaw Angles</h1>
                    <iframe src="01yaw_angles_chart.html" width="100%" height="500px"></iframe>
                    <h1>Time vs PID Throttle Output</h1>
                    <iframe src="02pid_throttle_output_chart.html" width="100%" height="500px"></iframe>
                </body>
            </html>
        """
        )


if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("csv_plot_node")

    # 获取参数
    csv_file_name = rospy.get_param("~csv_file", "boat_data_pid_test.csv")  # CSV 文件名
    output_path = rospy.get_param(
        "~output_path", "/home/shuai/ros_boat_ws/"
    )  # 输出路径

    try:
        rospy.loginfo("Starting to plot CSV data...")
        plot_csv_data(csv_file_name, output_path)
        rospy.loginfo("Plotting finished.")
    except rospy.ROSInterruptException:
        pass
