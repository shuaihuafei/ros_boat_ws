#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>

// 全局变量：目标的偏航角
float target_yaw_angle = 0;  // 期望的角度（单位为弧度）

// 无人艇当前状态
mavros_msgs::State current_state;

// 状态回调函数，更新无人艇的当前状态
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 回调函数，接收目标角度信息
void targetYawCallback(const std_msgs::Float32::ConstPtr& msg) {
    target_yaw_angle = msg->data;  // 获取目标偏航角
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4_angle_based_control_pid");
    ros::NodeHandle nh;

    // 订阅无人艇的当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    
    // 订阅目标的期望角度信息
    ros::Subscriber yaw_sub = nh.subscribe("/target_yaw_angle", 10, targetYawCallback);

    // 发布姿态控制指令
    ros::Publisher cmd_attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);

    // 解锁和设置模式的服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 设定控制频率为10Hz
    ros::Rate rate(10);

    // 等待飞控连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connected to PX4.");

    // 设置OFFBOARD模式的消息
    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    // 解锁无人艇的消息
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 发布初始的姿态指令，防止PX4因为没有接收到指令而无法切换到OFFBOARD模式
    geometry_msgs::PoseStamped cmd_attitude_msg;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_yaw_angle);  // 只调整偏航角，保持滚转和俯仰为0
    cmd_attitude_msg.pose.orientation.x = q.x();
    cmd_attitude_msg.pose.orientation.y = q.y();
    cmd_attitude_msg.pose.orientation.z = q.z();
    cmd_attitude_msg.pose.orientation.w = q.w();
    cmd_attitude_pub.publish(cmd_attitude_msg);

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        ros::spinOnce();  // 获取最新的状态和目标信息

        // 如果还没有进入OFFBOARD模式，且已经连接了PX4
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                ROS_INFO("Offboard mode enabled.");
            } else {
                ROS_WARN("Failed to set Offboard mode.");
            }
            last_request = ros::Time::now();
        }

        // 如果无人艇还没有解锁，且已经进入OFFBOARD模式
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed.");
            } else {
                ROS_WARN("Failed to arm vehicle.");
            }
            last_request = ros::Time::now();
        }

        // 生成目标的姿态四元数
        q.setRPY(0, 0, target_yaw_angle);  // 只调整偏航角（Yaw），保持滚转和俯仰为0
        cmd_attitude_msg.pose.orientation.x = q.x();
        cmd_attitude_msg.pose.orientation.y = q.y();
        cmd_attitude_msg.pose.orientation.z = q.z();
        cmd_attitude_msg.pose.orientation.w = q.w();

        // 发布姿态控制指令到 PX4
        cmd_attitude_pub.publish(cmd_attitude_msg);

        // 打印当前的目标偏航角
        ROS_INFO("Publishing target yaw: %f radians", target_yaw_angle);

        // 按照设定的频率运行
        rate.sleep();
    }

    return 0;
}
