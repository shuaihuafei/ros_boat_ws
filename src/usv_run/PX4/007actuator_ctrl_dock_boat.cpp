#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

typedef struct {
    float kp;
    float ki;
    float kd;
    float output_limit_l;
    float output_limit_r;
    float integral_limit;
} PID_t;

class PID {
public:
    explicit PID() {
        init();
    }

    float calculate(float current, float target, float dt) {
        if (std::isnan(current) || std::isnan(target) || std::isnan(dt) || dt <= 0) {
            return last_output;  // 返回上次输出
        }

        float error = target - current;

        // 计算积分项并限制范围
        integral += error * dt;
        integral = std::clamp(integral, -pid_t.integral_limit, pid_t.integral_limit);

        // 计算微分项
        float derivative = (error - previous_error) / dt;
        previous_error = error;

        // PID输出计算并限制范围
        float output = pid_t.kp * error + pid_t.ki * integral + pid_t.kd * derivative;
        output = std::clamp(output, pid_t.output_limit_l, pid_t.output_limit_r);

        last_output = output;
        return output;
    }

    void init() {
        pid_t.kp = 1.0f;
        pid_t.ki = 0.0f;
        pid_t.kd = 0.0f;
        pid_t.output_limit_l = 0.0f;
        pid_t.output_limit_r = 0.0f;
        pid_t.integral_limit = 0.0f;
        previous_error = integral = last_output = 0.0f;
    }

    void update(PID_t pid_t_) {
        pid_t.kp = pid_t_.kp;
        pid_t.ki = pid_t_.ki;
        pid_t.kd = pid_t_.kd;
        pid_t.output_limit_l = pid_t_.output_limit_l;
        pid_t.output_limit_r = pid_t_.output_limit_r;
        pid_t.integral_limit = pid_t_.integral_limit;
    }

private:
    PID_t pid_t;
    float previous_error;
    float integral;
    float last_output;
};


// 用于四元数转换的辅助函数
geometry_msgs::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    geometry_msgs::Quaternion q;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * sy + sr * cp * cy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// 用于四元数转换的辅助函数，将四元数转换为欧拉角（roll, pitch, yaw）
void quaternion_to_euler(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
    // 计算欧拉角
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // 夹角范围 [-pi/2, pi/2]
    else
        pitch = asin(sinp);

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

class AttitudeControlNode {
public:
    explicit AttitudeControlNode(ros::NodeHandle& nh) : nh_(nh) {
        // 从参数服务器获取参数
        nh_.param("kp_d", kp_d, 0.01);
        nh_.param("deg_th", deg_th, 10.0);
        nh_.param("pid_throttle_output", pid_throttle_output, 0.5);
        nh_.param("kp_boat_angle", kp_boat_angle, 1.0);
        ROS_INFO("kp_d: %f, kp_boat_angle: %f, deg_th: %f, pid_throttle_output: %f", kp_d, kp_boat_angle, deg_th, pid_throttle_output);

        // 获取PID参数
        nh_.param("kp_yaw", pid_yaw.kp, 1.0f);
        nh_.param("ki_yaw", pid_yaw.ki, 0.0f);
        nh_.param("kd_yaw", pid_yaw.kd, 0.0f);
        nh_.param("output_limit_yaw_l", pid_yaw.output_limit_l, -1.0f);
        nh_.param("output_limit_yaw_r", pid_yaw.output_limit_r, 1.0f);
        nh_.param("integral_limit_yaw", pid_yaw.integral_limit, 1.0f);
        ROS_INFO("pid_yaw.kp: %f", pid_yaw.kp);

        nh_.param("kp_throttle", pid_throttle.kp, 1.0f);
        nh_.param("ki_throttle", pid_throttle.ki, 0.0f);
        nh_.param("kd_throttle", pid_throttle.kd, 0.0f);
        nh_.param("output_limit_throttle_l", pid_throttle.output_limit_l, -1.0f);
        nh_.param("output_limit_throttle_r", pid_throttle.output_limit_r, 1.0f);
        nh_.param("integral_limit_throttle", pid_throttle.integral_limit, 1.0f);

        pid_yaw_ctrl.update(pid_yaw);
        pid_throttle_ctrl.update(pid_throttle);

        // 初始化订阅和发布
        state_sub = nh_.subscribe("/mavros/state", 10, &AttitudeControlNode::state_cb, this);
        actuator_control_sub = nh_.subscribe("/mavros/target_actuator_control", 10, &AttitudeControlNode::actuatorControlCallback, this);
        actuator_control_pub = nh_.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
        pose_sub = nh_.subscribe("/mavros/local_position/pose", 10, &AttitudeControlNode::poseCallback, this);
        local_velocity_body_sub = nh_.subscribe("/mavros/local_position/velocity_body", 10, &AttitudeControlNode::localVelocityBodyCallback, this);
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        timer_dock = nh_.createTimer(ros::Duration(1.0), &AttitudeControlNode::checkDockTimeout, this);
        timer_boat = nh_.createTimer(ros::Duration(1.0), &AttitudeControlNode::checkBoatTimeout, this);
        dock_angle_sub = nh.subscribe("/yolo/dock_angle", 10, &AttitudeControlNode::dockAngleCallback, this);
        boat_angle_sub = nh.subscribe("/yolo/boat_angle", 10, &AttitudeControlNode::boatAngleCallback, this);

        flag_yaw = true;
        flag_v_x = true;
        last_received_time_dock = ros::Time::now();
        last_received_time_boat = ros::Time::now();
        current_yaw = 0.0;
        desired_yaw = 0.0;
        des_cur_error = 0.0;
        dock_angle = 0.0;
        boat_angle = 0.0;
        current_v_body_x = 0.0;
        update_counter = 0;
        control_signal_2 = 0.0;
        control_signal_3 = 0.0;
        pid_yaw_output = 0.0;

        // 这个CSV文件用来记录实验数据
        output_file_pid.open("/home/shuai/ros_boat_ws/boat_data_pid.csv", std::ios::app);
        if (output_file_pid.is_open()) {
            output_file_pid << "time,position_x,position_y,position_z,yaw_angle,v_body_x,dock_angle,boat_angle,current_yaw,desired_yaw,des_cur_error,kp_d,deg_th,pid_yaw_output,pid_throttle_output,control_signal_2,control_signal_3\n";
        } else {
            ROS_ERROR("Failed to open CSV file for logging.");
        }
        
        // 这个CSV文件用来调试PID
        output_file_pid_test.open("/home/shuai/ros_boat_ws/boat_data_pid_test.csv", std::ios::out);
        if (output_file_pid_test.is_open()) {
            output_file_pid_test << "time,position_x,position_y,position_z,yaw_angle,v_body_x,dock_angle,boat_angle,current_yaw,desired_yaw,des_cur_error,kp_d,deg_th,pid_yaw_output,pid_throttle_output,control_signal_2,control_signal_3\n";
        } else {
            ROS_ERROR("Failed to open CSV file for logging.");
        }
    }

    ~AttitudeControlNode() {
        if (output_file_pid.is_open()) {
            output_file_pid.close();
        }
        if (output_file_pid_test.is_open()) {
            output_file_pid_test.close();
        }
    }

    void updateActuatorControl() {
        if(dock_angle != -1)
            desired_yaw = -1 * (dock_angle + kp_boat_angle * boat_angle) * M_PI / 180.0 + current_yaw_record;
    }

    // 电机控制
    void sendActuatorControl() {
        if(dock_angle != -1)
        {
            // ros::Time now = ros::Time::now();
            // double dt = (now - last_update_time).toSec();
            // last_update_time = now;
            double dt = 0.01;

            pid_yaw_output = -1 * pid_yaw_ctrl.calculate(current_yaw, desired_yaw, dt);
            des_cur_error = desired_yaw - current_yaw;
            ROS_INFO_THROTTLE(1, "error angle: %f deg", des_cur_error);

            // if(fabs(pid_throttle_output - 0.0) < 0.1)
            pid_throttle_output = 0.5;

            // pid_throttle_output = kp_d * (deg_th - fabs(dock_angle)) * pid_throttle_output + pid_throttle_output;
            // if(pid_throttle_output < -1){
            //     pid_throttle_output = -1;
            // }
            // else if(pid_throttle_output > 1){
            //     pid_throttle_output = 1;
            // }
            // pid_throttle_output = pid_throttle_ctrl.calculate(current_v_body_x, desired_v_body_x, dt);

            ROS_INFO_THROTTLE(1, "pid_yaw_output: %f", pid_yaw_output);
            ROS_INFO_THROTTLE(1, "pid_throttle_output: %f", pid_throttle_output);

            mavros_msgs::ActuatorControl actuator_control_msg;
            // 设置电机控制值
            actuator_control_msg.controls[0] = 0.0;
            actuator_control_msg.controls[1] = 0.0;
            actuator_control_msg.controls[2] = pid_yaw_output;
            actuator_control_msg.controls[3] = pid_throttle_output;
            actuator_control_msg.controls[4] = 0.0;
            actuator_control_msg.controls[5] = 0.0;
            actuator_control_msg.controls[6] = 0.0;
            actuator_control_msg.controls[7] = 0.0;

            actuator_control_msg.group_mix = 0;  // 控制组
            // 发布控制信号
            actuator_control_pub.publish(actuator_control_msg);
        }
        else{

            // if(current_yaw < M_PI)
            // {
                pid_yaw_output = -1;
                pid_throttle_output = 0;
            // }
            // else if(current_yaw <= 2 * M_PI){
            //     pid_yaw_output = 1;
            //     pid_throttle_output = 0;
            // }

            mavros_msgs::ActuatorControl actuator_control_msg;
            // 设置电机控制值
            actuator_control_msg.controls[0] = 0.0;
            actuator_control_msg.controls[1] = 0.0;
            actuator_control_msg.controls[2] = pid_yaw_output;
            actuator_control_msg.controls[3] = pid_throttle_output;
            actuator_control_msg.controls[4] = 0.0;
            actuator_control_msg.controls[5] = 0.0;
            actuator_control_msg.controls[6] = 0.0;
            actuator_control_msg.controls[7] = 0.0;

            actuator_control_msg.group_mix = 0;  // 控制组
            // 发布控制信号
            actuator_control_pub.publish(actuator_control_msg);
        }
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    // 点击输出值获取
    void actuatorControlCallback(const mavros_msgs::ActuatorControl::ConstPtr& msg) {
        // 检查 msg->controls 数组是否包含足够的元素
        if (msg->controls.size() >= 4) {
            // 获取第3个（索引为2）和第4个（索引为3）的控制信号
            control_signal_2 = msg->controls[2];
            control_signal_3 = msg->controls[3];

            // 打印原始控制信号
            ROS_INFO_THROTTLE(1, "Actuator 2 control signal: %f", control_signal_2);
            ROS_INFO_THROTTLE(1, "Actuator 3 control signal: %f", control_signal_3);
        } else {
            ROS_WARN("Received actuator control feedback, but controls array is too small.");
        }
    }

    // 姿态获取
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 获取位置
        const geometry_msgs::Point& position = msg->pose.position;

        // 获取四元数方向
        const geometry_msgs::Quaternion& orientation = msg->pose.orientation;

        // 转换四元数为欧拉角
        double roll, pitch, yaw;
        quaternion_to_euler(orientation, roll, pitch, yaw);

        if(yaw < 0)
        {
            yaw = yaw + 2 * M_PI;
        }
        
        current_yaw = yaw;

        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        double yaw_deg = yaw * 180.0 / M_PI;

        // 更新当前yaw值
        if(flag_yaw)
        {
            current_yaw_record = yaw;
            // flag_yaw = false;
        }

        // 打印位置和欧拉角
        ROS_INFO_THROTTLE(1, "Position: [x: %f, y: %f, z: %f]", position.x, position.y, position.z);
        ROS_INFO_THROTTLE(1, "Orientation (Euler angles in degrees): roll: %f, pitch: %f, yaw: %f", roll_deg, pitch_deg, yaw_deg);

        // 存储记录值
        position_x = position.x;
        position_y = position.y;
        position_z = position.z;
        roll_angle = roll_deg;
        pitch_angle = pitch_deg;
        yaw_angle = yaw_deg;
    }

    // 定时器回调函数，用于检测消息超时
    void checkDockTimeout(const ros::TimerEvent&) {
        // 如果超过 5 秒没有接收到消息，设置 dock_angle 为 0
        if ((ros::Time::now() - last_received_time_dock).toSec() > 5.0) {
            dock_angle = -1;
            ROS_WARN("No dock angle received for 1 second, setting dock_angle to -1.");
        }
    }

    // 定时器回调函数，用于检测消息超时
    void checkBoatTimeout(const ros::TimerEvent&) {
        // 如果超过 5 秒没有接收到消息，设置 dock_angle 为 0
        if ((ros::Time::now() - last_received_time_boat).toSec() > 2.0) {
            boat_angle = 0;
            ROS_WARN("No boat angle received for 1 second, setting boat_angle to -1.");
        }
    }

    // 视觉获取坞舱角度
    void dockAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
        dock_angle = msg->data;  // 获取目标偏航角
        last_received_time_dock = ros::Time::now();  // 更新接收时间
        ROS_INFO_THROTTLE(1, "Received target yaw angle: %f deg", dock_angle);
    }

    // 视觉获取无人艇角度
    void boatAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
        boat_angle = msg->data;  // 获取目标偏航角
        last_received_time_boat = ros::Time::now();  // 更新接收时间
        ROS_INFO_THROTTLE(1, "Received target yaw angle: %f deg", dock_angle);
    }

    // 体坐标系FLU速度获取
    void localVelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        // 提取机体坐标系的速度信息
        v_body_x = msg->twist.linear.x;
        v_body_y = msg->twist.linear.y;
        v_body_z = msg->twist.linear.z;
        w_body_x = msg->twist.angular.x;
        w_body_y = msg->twist.angular.y;
        w_body_z = msg->twist.angular.z;

        current_v_body_x = v_body_x;

        if(flag_v_x)
        {
            current_v_body_x_record = v_body_x;
            // flag_v_x = false;
        }

        // 打印线速度和角速度
        ROS_INFO_THROTTLE(1, "Body Linear Velocity: [x: %f, y: %f, z: %f]", v_body_x, v_body_y, v_body_z);
        ROS_INFO_THROTTLE(1, "Body angular Velocity: [x: %f, y: %f, z: %f]", w_body_x, w_body_y, w_body_z);

    }

    void setOffboardMode() {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.base_mode = 0;  // 0表示自定义模式（不影响现有模式）
        mode_cmd.request.custom_mode = "OFFBOARD";  // 设置模式为 Offboard

        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
            ROS_INFO("Offboard mode set successfully.");
        } else {
            ROS_ERROR("Failed to set Offboard mode.");
        }
    }

    void armDrone() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;  // 请求解锁

        if (arming_client.call(arm_cmd)) {
            if (arm_cmd.response.success) {
                ROS_INFO("Drone armed successfully.");
            } else {
                ROS_ERROR("Failed to arm drone.");
            }
        } else {
            ROS_ERROR("Failed to call arm service.");
        }
    }

    std::string getCurrentTimeString() {
        // 获取当前系统时间
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        // 将当前时间转换为std::time_t类型
        std::time_t timeT = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&timeT);

        // 使用ostringstream格式化为"YYYY-MM-DD HH:MM:SS.mmm"
        std::ostringstream timeStream;
        timeStream << std::put_time(&tm, "%Y-%m-%d=%H:%M:%S") 
                << "." 
                << std::setw(3) << std::setfill('0') << ms.count();
        
        return timeStream.str();
    }

    void run() {
        ros::Rate rate(20.0);

        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }
        for(int i = 100; ros::ok() && i > 0; --i){
            sendActuatorControl();
            ros::spinOnce();
            rate.sleep();
        }

        // 切换到 Offboard 模式并解锁
        setOffboardMode();
        armDrone();

        // 一旦模式切换并解锁，就开始发布控制指令
        while (ros::ok()) {
            sendActuatorControl();

            std::string currentTime = getCurrentTimeString();

            if (update_counter % 10 == 0) {
                updateActuatorControl();
                update_counter = 0;
                
                output_file_pid << currentTime << ","
                            << position_x << ","
                            << position_y << ","
                            << position_z << ","
                            << yaw_angle << ","
                            << v_body_x << ","
                            << dock_angle << ","
                            << boat_angle << ","
                            << (current_yaw * 180.0 / M_PI) << ","
                            << (desired_yaw * 180.0 / M_PI) << ","
                            << des_cur_error << ","
                            << kp_d << ","
                            << deg_th << ","
                            << pid_yaw_output << ","
                            << pid_throttle_output << ","
                            << control_signal_2 << ","
                            << control_signal_3 << "\n";
                // 记录到PID调试CSV文件
                output_file_pid_test << currentTime << ","
                            << position_x << ","
                            << position_y << ","
                            << position_z << ","
                            << yaw_angle << ","
                            << v_body_x << ","
                            << dock_angle << ","
                            << boat_angle << ","
                            << (current_yaw * 180.0 / M_PI) << ","
                            << (desired_yaw * 180.0 / M_PI) << ","
                            << des_cur_error << ","
                            << kp_d << ","
                            << deg_th << ","
                            << pid_yaw_output << ","
                            << pid_throttle_output << ","
                            << control_signal_2 << ","
                            << control_signal_3 << "\n";
            }
            update_counter++;

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub, actuator_control_sub, pose_sub, local_velocity_body_sub, dock_angle_sub, boat_angle_sub;
    ros::Publisher actuator_control_pub;
    ros::ServiceClient set_mode_client, arming_client;

    PID_t pid_yaw{}, pid_throttle{};

    PID pid_yaw_ctrl;
    PID pid_throttle_ctrl;
    mavros_msgs::State current_state;

    double kp_d, deg_th, kp_boat_angle;
    double current_yaw, current_yaw_record, current_v_body_x, current_v_body_x_record;
    double des_cur_error;
    double dock_angle, boat_angle;
    double desired_yaw, desired_v_body_x;
    double pid_yaw_output;
    double pid_throttle_output;
    bool flag_yaw, flag_v_x;
    int update_counter;
    float control_signal_2, control_signal_3;

    double position_x, position_y, position_z, roll_angle, pitch_angle, yaw_angle;
    double v_body_x, v_body_y, v_body_z, w_body_x, w_body_y, w_body_z;
    ros::Time last_update_time;
    std::ofstream output_file_pid, output_file_pid_test;
    ros::Timer timer_dock, timer_boat;  // 定时器，用于检查超时
    ros::Time last_received_time_dock, last_received_time_boat;  // 上次接收到目标角度的时间
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "attitude_control_node");

    ros::NodeHandle nh("~");

    AttitudeControlNode attitudeControlNode(nh);
    attitudeControlNode.run();

    return 0;
}
