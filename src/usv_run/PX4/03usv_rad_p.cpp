#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

// 全局变量：记录当前无人船状态和目标偏航角
mavros_msgs::State current_state;
float target_yaw_angle = 0.0;  // 期望的角度（单位：弧度）
ros::Time last_detection_time; // 最近检测到目标的时间

// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 角度信息回调函数
void objectAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    target_yaw_angle = msg->data;  // 获取目标偏航角
    last_detection_time = ros::Time::now();  // 更新最近检测到目标的时间
    ROS_INFO("Received target yaw angle: %f radians", target_yaw_angle);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "boat_run_with_yaw_control");
    ros::NodeHandle nh;

    // 订阅无人船的当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // 订阅目标角度信息
    ros::Subscriber yaw_sub = nh.subscribe("/object_angle", 10, objectAngleCallback);

    // 创建发布速度指令的发布者
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // 创建服务客户端，用于解锁和设置模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 等待ROS和飞控连接
    ros::Rate rate(20);
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Boat connected.");

    // 创建解锁和模式设置的请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD"; 

    // 记录请求时间
    ros::Time last_request = ros::Time::now();

    // 初始化速度指令
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.y = 0.5;  // 直走速度

    bool armed = false;          // 标记是否已解锁
    bool offboard_mode = false;  // 标记是否已进入OFFBOARD模式

    // 控制循环
    while (ros::ok()) {
        // 检查是否超过10秒未检测到目标
        if ((ros::Time::now() - last_detection_time).toSec() > 10.0) {
            // 如果10秒未检测到目标，则停止转向
            cmd_vel_msg.linear.x = 0.0;
            ROS_WARN("No target detected for 10 seconds. Stopping rotation.");
        } else {
            // 根据目标角度调整转向速度
            cmd_vel_msg.linear.x = target_yaw_angle;  // 使用线速度控制转向
        }

        // 持续发布速度指令
        cmd_vel_pub.publish(cmd_vel_msg);

        // 尝试解锁
        if (!armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Boat armed successfully.");
                armed = true;  // 解锁成功
            } else {
                ROS_ERROR("Failed to arm the boat.");
            }
            last_request = ros::Time::now();  // 更新请求时间
        }

        // 解锁成功后，切换到OFFBOARD模式
        if (armed && !offboard_mode && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD mode set successfully.");
                offboard_mode = true;  // OFFBOARD模式设置成功
            } else {
                ROS_ERROR("Failed to set OFFBOARD mode.");
            }
            last_request = ros::Time::now();  // 更新请求时间
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
