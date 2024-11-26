#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <math.h>
#include <fstream>  // 包含文件操作库

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
    AttitudeControlNode(ros::NodeHandle& nh):nh_(nh)  {
        // 获取参数，若未定义，使用默认值
        nh_.param("input_deg", input_deg, 60.0);  // 默认值 60.0
        nh_.param("input_thrust", input_thrust, 0.5);  // 默认值 0.5

        ROS_INFO("Input degrees: %f, Input thrust: %f", input_deg, input_thrust);

        // 初始化yaw为0
        current_yaw = 0.0;
        flag = true;
        // 初始化变量
        control_signal_2_in_degrees = 0.0;
        record_counter = 0;

        state_sub = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &AttitudeControlNode::state_cb, this);
        // 发布器，用于发送姿态控制指令
        attitude_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        // 订阅器，用于获取混合器反馈
        actuator_sub = nh_.subscribe("/mavros/target_actuator_control", 10, &AttitudeControlNode::actuatorControlCallback, this);
        // 订阅器，用于获取本地位置和姿态
        pose_sub = nh_.subscribe("/mavros/local_position/pose", 10, &AttitudeControlNode::poseCallback, this);
        // 发布器，用于设置飞行模式
        set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        // 发布器，用于解锁
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        // 打开CSV文件用于记录
        output_file.open("/home/shuai/ros_boat_ws/boat_data.csv", std::ios::app);
        if (output_file.is_open()) {
            // 写入CSV文件的标题行
            output_file << "Time,Position_X,Position_Y,Position_Z,Roll,Pitch,Yaw,Input_Degrees,Input_Thrust,Control_Signal_2,Control_Signal_3,Control_Signal_2_Degrees\n";
        } else {
            ROS_ERROR("Failed to open CSV file for logging.");
        }
    }

    ~AttitudeControlNode() {
        // 关闭文件流
        if (output_file.is_open()) {
            output_file.close();
        }
    }

    void sendAttitudeControl() {
        mavros_msgs::AttitudeTarget attitude_target;

        // 设置 type_mask (仅控制姿态，不控制推力和角速度)
        attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        // 设置目标姿态（四元数表示）
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = input_deg * M_PI / 180.0 + current_yaw;  // 当前yaw值加上pose中的yaw

        // 获取四元数
        attitude_target.orientation = euler_to_quaternion(roll, pitch, yaw);

        // 设置角速度（这里设为0）
        attitude_target.body_rate.x = 0.0;
        attitude_target.body_rate.y = 0.0;
        attitude_target.body_rate.z = 0.0;

        // 设置推力（这里设为0.5）
        attitude_target.thrust = input_thrust;

        // 发布姿态控制指令
        attitude_pub.publish(attitude_target);
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void actuatorControlCallback(const mavros_msgs::ActuatorControl::ConstPtr& msg) {
        // 检查 msg->controls 数组是否包含足够的元素
        if (msg->controls.size() >= 4) {
            // 获取第3个（索引为2）和第4个（索引为3）的控制信号
            control_signal_2 = msg->controls[2];
            control_signal_3 = msg->controls[3];

            // 打印原始控制信号
            ROS_INFO("Actuator 2 control signal: %f", control_signal_2);
            ROS_INFO("Actuator 3 control signal: %f", control_signal_3);

            // 将第3个控制信号乘以180，假设控制信号代表一个角度值
            control_signal_2_in_degrees = control_signal_2 * 180.0;
            ROS_INFO("Actuator 2 control signal in degrees: %f", control_signal_2_in_degrees);
        } else {
            ROS_WARN("Received actuator control feedback, but controls array is too small.");
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // 获取位置
        const geometry_msgs::Point& position = msg->pose.position;

        // 获取四元数方向
        const geometry_msgs::Quaternion& orientation = msg->pose.orientation;

        // 转换四元数为欧拉角
        double roll, pitch, yaw;
        quaternion_to_euler(orientation, roll, pitch, yaw);

        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        double yaw_deg = yaw * 180.0 / M_PI;

        // 更新当前yaw值
        if(flag)
        {
            current_yaw = yaw;
            // flag = false;
        }

        // 打印位置和欧拉角
        ROS_INFO("Position: [x: %f, y: %f, z: %f]", position.x, position.y, position.z);
        ROS_INFO("Orientation (Euler angles in degrees): roll: %f, pitch: %f, yaw: %f", roll_deg, pitch_deg, yaw_deg);

        // 存储记录值
        position_x = position.x;
        position_y = position.y;
        position_z = position.z;
        roll_angle = roll_deg;
        pitch_angle = pitch_deg;
        yaw_angle = yaw_deg;
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

    void run() {
        ros::Rate rate(20.0);

        while(ros::ok() && !current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }
        for(int i = 100; ros::ok() && i > 0; --i){
            sendAttitudeControl();
            ros::spinOnce();
            rate.sleep();
        }

        // 切换到 Offboard 模式并解锁
        setOffboardMode();
        armDrone();

        // 一旦模式切换并解锁，就开始发布控制指令
        while (ros::ok()) {
            sendAttitudeControl();

            if (record_counter % 10 == 0) {
                record_counter = 0;
                // 获取当前时间
                double time = ros::Time::now().toSec();
                // 记录到CSV文件
                output_file << time << ","
                            << position_x << ","
                            << position_y << ","
                            << position_z << ","
                            << roll_angle << ","
                            << pitch_angle << ","
                            << yaw_angle << ","
                            << input_deg << ","
                            << input_thrust << ","
                            << control_signal_2 << ","
                            << control_signal_3 << ","
                            << control_signal_2_in_degrees << "\n";
            }

            record_counter++;
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub;
    ros::Publisher attitude_pub;
    ros::Subscriber actuator_sub;
    ros::Subscriber pose_sub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;
    mavros_msgs::State current_state;

    // 存储数据
    double current_yaw;
    bool flag;
    double position_x, position_y, position_z;
    double roll_angle, pitch_angle, yaw_angle;
    double control_signal_2, control_signal_3;
    double control_signal_2_in_degrees;
    double input_deg;
    double input_thrust;
    int record_counter;
    std::ofstream output_file;  // 文件输出流
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "attitude_control_node");

    ros::NodeHandle nh("~");

    AttitudeControlNode attitudeControlNode(nh);
    attitudeControlNode.run();

    return 0;
}
