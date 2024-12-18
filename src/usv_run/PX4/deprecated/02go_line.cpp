#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

// 记录当前无人船状态
mavros_msgs::State current_state;

// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "boat_run");
    ros::NodeHandle nh;

    // 读取来自参数服务器的初始线性速度 z 值
    double linear_z_speed;
    nh.param("linear_z_speed", linear_z_speed, 0.5);  // 默认值为 0.5

    // 订阅无人船的当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // 创建发布速度指令的发布者
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    // 创建服务客户端，用于解锁和设置模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 等待ROS和飞控连接
    ros::Rate rate(20);  // 增加频率以确保足够的速度指令发送
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

    // 持续发布速度指令
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = -5; // 从参数服务器获取的速度值

    bool armed = false;          // 标记是否已解锁
    bool offboard_mode = false;  // 标记是否已进入OFFBOARD模式

    // 控制循环
    while (ros::ok()) {
        // 持续发布速度指令（即使未进入OFFBOARD模式）
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
