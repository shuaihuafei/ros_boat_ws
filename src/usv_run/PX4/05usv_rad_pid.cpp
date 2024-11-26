#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

// 全局变量：记录当前无人船状态和两个目标偏航角差值
mavros_msgs::State current_state;
float yaw_error = 0.0;             // 摄像头传来的无人艇与目标方向之间的角度差值
float dock_angle_error = 0.0;      // 船坞角度的角度差值
float error_sum_yaw = 0.0;         // 摄像头角度的积分误差累加
float error_sum_dock = 0.0;        // 船坞角度的积分误差累加
float last_error_yaw = 0.0;        // 上一次摄像头角度误差
float last_error_dock = 0.0;       // 上一次船坞角度误差
ros::Time last_detection_time;     // 最近检测到目标的时间

// PID参数（从参数服务器读取）
// 第一套 PID 参数（用于摄像头角度控制）
float kp_yaw, ki_yaw, kd_yaw;
// 第二套 PID 参数（用于船坞角度控制）
float kp_dock, ki_dock, kd_dock;

// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 摄像头角度差值信息回调函数
void objectAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    yaw_error = msg->data;  // 获取摄像头传来的角度差值
    last_detection_time = ros::Time::now();  // 更新最近检测到目标的时间
    ROS_INFO("Received yaw error: %f radians", yaw_error);
}

// 船坞角度差值信息回调函数
void dockAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    dock_angle_error = msg->data;  // 获取船坞的角度差值
    ROS_INFO("Received dock angle error: %f radians", dock_angle_error);
}

// PID控制器函数，适用于两套角度控制
float calculatePIDControl(float error, float& error_sum, float& last_error, float kp, float ki, float kd) {
    // 将误差保持在 [-pi, pi] 范围内，防止角度跳跃
    if (error > M_PI) error -= 2 * M_PI;
    if (error < -M_PI) error += 2 * M_PI;

    // 误差累加（积分项）
    error_sum += error;

    // 计算误差变化率（微分项）
    float error_rate = error - last_error;

    // 计算PID输出
    float control_output = kp * error + ki * error_sum + kd * error_rate;

    // 更新上一次的误差
    last_error = error;

    return control_output;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "boat_run_with_yaw_control_pid");
    ros::NodeHandle nh;

    // 从参数服务器读取第一套PID参数（用于摄像头角度控制）
    nh.param("kp_yaw", kp_yaw, 0.5f);
    nh.param("ki_yaw", ki_yaw, 0.1f);
    nh.param("kd_yaw", kd_yaw, 0.05f);

    // 从参数服务器读取第二套PID参数（用于船坞角度控制）
    nh.param("kp_dock", kp_dock, 0.5f);
    nh.param("ki_dock", ki_dock, 0.1f);
    nh.param("kd_dock", kd_dock, 0.05f);

    // 订阅无人船的当前状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // 订阅角度差值信息
    ros::Subscriber yaw_sub = nh.subscribe("/object_angle", 10, objectAngleCallback);
    ros::Subscriber dock_sub = nh.subscribe("/dock_angle", 10, dockAngleCallback);

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
            // 使用两套PID控制器分别计算两个转向控制量
            float control_yaw = calculatePIDControl(yaw_error, error_sum_yaw, last_error_yaw, kp_yaw, ki_yaw, kd_yaw);
            float control_dock = calculatePIDControl(dock_angle_error, error_sum_dock, last_error_dock, kp_dock, ki_dock, kd_dock);

            // 合并两个控制量作为最终的转向控制
            cmd_vel_msg.linear.x = control_yaw + control_dock;
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
