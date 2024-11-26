#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

// 全局变量存储无人艇状态
mavros_msgs::State current_state;

// 状态回调函数，用于更新当前飞控状态
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 切换到 GUIDED 模式
bool setGuidedMode(ros::NodeHandle& nh) {
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "GUIDED";

    if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("GUIDED mode set successfully.");
        return true;
    } else {
        ROS_ERROR("Failed to set GUIDED mode.");
        return false;
    }
}

// 解锁无人艇
bool armVehicle(ros::NodeHandle& nh) {
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed successfully.");
        return true;
    } else {
        ROS_ERROR("Failed to arm vehicle.");
        return false;
    }
}

// 设置目标姿态（航向角度）
void setDesiredYaw(ros::Publisher& attitude_pub, float yaw_angle_deg) {
    geometry_msgs::PoseStamped attitude_msg;

    // 设置姿态消息的时间戳
    attitude_msg.header.stamp = ros::Time::now();
    attitude_msg.header.frame_id = "base_link";

    // 将角度从度转换为弧度
    float yaw_angle_rad = yaw_angle_deg * M_PI / 180.0;

    // 使用四元数表示目标航向角
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_angle_rad);  // 假设无人艇是二维的，只有航向角（Yaw）
    attitude_msg.pose.orientation.x = q.x();
    attitude_msg.pose.orientation.y = q.y();
    attitude_msg.pose.orientation.z = q.z();
    attitude_msg.pose.orientation.w = q.w();

    // 发布目标姿态消息
    attitude_pub.publish(attitude_msg);

    ROS_INFO("Setting target yaw: %.2f degrees", yaw_angle_deg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "boat_attitude_control");
    ros::NodeHandle nh;

    // 订阅无人艇状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);

    // 发布姿态控制指令
    ros::Publisher attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);

    ros::Rate rate(20.0);

    // 等待与飞控建立连接
    while (ros::ok() && !current_state.connected) {
        ROS_INFO("Waiting for connection to flight controller...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to flight controller.");

    // 切换到 GUIDED 模式
    if (!setGuidedMode(nh)) {
        return -1;
    }

    // 解锁无人艇
    if (!armVehicle(nh)) {
        return -1;
    }

    // 设置初始目标航向角
    float target_yaw_deg = 45.0;  // 目标航向，单位：度

    // 主循环
    while (ros::ok()) {
        // 检查飞控是否在 GUIDED 模式
        if (current_state.mode != "GUIDED") {
            ROS_WARN("Not in GUIDED mode, trying to set again...");
            if (!setGuidedMode(nh)) {
                rate.sleep();
                continue;
            }
        }

        // 发送目标姿态（航向角度）
        setDesiredYaw(attitude_pub, target_yaw_deg);

        // 如果需要，可以动态调整目标航向角
        // target_yaw_deg += 1.0;  // 示例：每次循环增加目标航向角

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
