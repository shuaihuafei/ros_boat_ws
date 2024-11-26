#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>

// 定义 π 常量
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 获取当前时间（以毫秒为单位）
uint32_t millis() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return static_cast<uint32_t>(duration.count());
}

// PID 控制类
class PID {
public:
    PID(float kp, float ki, float kd, float imax = 0.0f, float fCut = 20.0f)
        : _kp(kp), _ki(ki), _kd(kd), _imax(imax), _fCut(fCut),
          _integrator(0), _last_error(0), _last_derivative(NAN), _last_t(0) {}

    float get_pid(float error, float scaler = 1.0f) {
        uint32_t tnow = millis(); // 获取当前时间
        uint32_t dt = tnow - _last_t;
        float output = 0;
        float delta_time;

        if (_last_t == 0 || dt > 1000) {
            dt = 0;
            reset_I();
        }
        _last_t = tnow;

        delta_time = static_cast<float>(dt) * 0.001f; // 转换为秒

        // 计算比例项
        output += error * _kp;

        // 计算微分项
        if (_kd != 0.0f && dt > 0) {
            float derivative;
            if (std::isnan(_last_derivative)) {
                derivative = 0.0f;
                _last_derivative = 0.0f;
            } else {
                derivative = (error - _last_error) / delta_time;
            }

            // 离散低通滤波
            float RC = 1 / (2 * M_PI * _fCut);
            derivative = _last_derivative + 
                         (delta_time / (RC + delta_time)) * (derivative - _last_derivative);

            _last_derivative = derivative;
            _last_error = error;

            output += _kd * derivative;
        }

        // 计算积分项
        if (_ki != 0.0f && dt > 0) {
            _integrator += error * _ki * delta_time;
            if (_integrator > _imax) {
                _integrator = _imax;
            } else if (_integrator < -_imax) {
                _integrator = -_imax;
            }
            output += _integrator;
        }

        // 缩放输出
        output *= scaler;
        output = std::clamp(output, -1.0f, 1.0f);

        return output;
    }

    void reset_I() {
        _integrator = 0.0f;
        _last_derivative = NAN;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _imax;
    float _fCut;
    float _integrator;
    float _last_error;
    float _last_derivative;
    uint32_t _last_t;
};

// Yaw 控制节点类
class YawControlNode {
public:
    explicit YawControlNode(ros::NodeHandle& nh) 
        : nh_(nh), target_angle_(0.0f), current_angle_(0.0f),
          fixed_thrust_(0.5f), yaw_pid_(0.5f, 0.1f, 0.01f, 10.0f, 20.0f) {
        // 初始化订阅和发布
        target_angle_sub_ = nh_.subscribe("/object_angle", 10, &YawControlNode::objectAngleCallback, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &YawControlNode::poseCallback, this);
        state_sub_ = nh_.subscribe("/mavros/state", 10, &YawControlNode::stateCallback, this);
        actuator_control_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);

        ROS_INFO("Yaw Control Node initialized.");
    }

    void run() {
        ros::Rate rate(20.0); // 控制频率

        // 主循环，实时运行 PID 控制逻辑
        while (ros::ok()) {
            ros::spinOnce();

            // 使用最新的目标角度和当前姿态计算误差
            float error = target_angle_ - current_angle_;

            // 使用 PID 控制器计算 yaw 输出
            float yaw_output = -1 * yaw_pid_.get_pid(error);

            // 发布 actuator 控制
            publishActuatorControl(yaw_output);

            ROS_INFO("Yaw control output: %.2f, Target angle: %.2f, Current angle: %.2f", 
                     yaw_output, target_angle_, current_angle_);

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_angle_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher actuator_control_pub_;

    mavros_msgs::State current_state_;
    float target_angle_;     // 当前目标角度
    float current_angle_;    // 当前航向角
    float fixed_thrust_;     // 固定推力
    PID yaw_pid_;            // PID 控制器

    // 从视觉模块更新目标角度
    void objectAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
        float cam_yaw_angle = msg->data; // 从消息中获取摄像头角度
        target_angle_ = -1 * cam_yaw_angle * M_PI / 180.0 + current_angle_; // 根据当前航向修正目标
        ROS_INFO("Updated target angle from camera: %.2f radians", target_angle_);
    }

    // 当前姿态回调函数，从 /mavros/local_position/pose 提取 yaw
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 提取四元数
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        // 转换为欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        current_angle_ = yaw; // 当前角度为弧度
    }

    // 飞控状态回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    // 发布 actuator 控制指令
    void publishActuatorControl(float yaw_output) {
        mavros_msgs::ActuatorControl actuator_msg;
        actuator_msg.header.stamp = ros::Time::now();
        actuator_msg.group_mix = 0; // 使用默认的控制组

        actuator_msg.controls[2] = yaw_output; // 假定索引 2 是 YAW
        actuator_msg.controls[3] = fixed_thrust_; // 假定索引 3 是 THROTTLE

        for (int i = 0; i < 8; i++) {
            if (i != 2 && i != 3) {
                actuator_msg.controls[i] = NAN;
            }
        }

        actuator_control_pub_.publish(actuator_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yaw_control_node");

    ros::NodeHandle nh("~");
    YawControlNode yaw_control_node(nh);
    yaw_control_node.run();

    return 0;
}
