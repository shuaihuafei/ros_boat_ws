#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <fstream>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/ros.h>

// 定义一个全局的输出文件流
std::ofstream csv_file;

// 定义一个结构体存储一行数据
struct Data {
    double pos_x, pos_y, pos_z;
    double qx, qy, qz, qw;
    double v_body_x, v_body_y, v_body_z;
    double w_body_x, w_body_y, w_body_z;
    double v_enu_x, v_enu_y, v_enu_z;
    double w_enu_x, w_enu_y, w_enu_z;
    bool position_received = false;
    bool velocity_body_received = false;
    bool velocity_local_received = false;
};

// 用一个全局的 Data 变量来存储接收到的数据
Data data;

// 记录所有数据的回调函数

void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 提取位置和姿态信息
    data.pos_x = msg->pose.position.x;
    data.pos_y = msg->pose.position.y;
    data.pos_z = msg->pose.position.z;
    data.qx = msg->pose.orientation.x;
    data.qy = msg->pose.orientation.y;
    data.qz = msg->pose.orientation.z;
    data.qw = msg->pose.orientation.w;

    data.position_received = true;

    // 创建四元数对象
    tf2::Quaternion quat(data.qx, data.qy, data.qz, data.qw);

    // 转换为欧拉角（roll, pitch, yaw）
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // 打印位置信息和转换后的欧拉角
    ROS_INFO("Position: x = %.3f, y = %.3f, z = %.3f", data.pos_x, data.pos_y, data.pos_z);
    ROS_INFO("Orientation (Euler Angles): Roll = %.3f, Pitch = %.3f, Yaw = %.3f", roll, pitch, yaw);

    // 如果所有信息都已接收到，写入 CSV
    if (data.position_received && data.velocity_body_received && data.velocity_local_received) {
        csv_file << data.pos_x << ", " << data.pos_y << ", " << data.pos_z << ", "
                 << data.qx << ", " << data.qy << ", " << data.qz << ", " << data.qw << ", "
                 << data.v_body_x << ", " << data.v_body_y << ", " << data.v_body_z << ", "
                 << data.w_body_x << ", " << data.w_body_y << ", " << data.w_body_z << ", "
                 << data.v_enu_x << ", " << data.v_enu_y << ", " << data.v_enu_z << ", "
                 << data.w_enu_x << ", " << data.w_enu_y << ", " << data.w_enu_z << std::endl;
        
        // 重置标记，准备接收下一组数据
        data.position_received = false;
        data.velocity_body_received = false;
        data.velocity_local_received = false;
    }
}

void localVelocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // 提取机体坐标系的速度信息
    data.v_body_x = msg->twist.linear.x;
    data.v_body_y = msg->twist.linear.y;
    data.v_body_z = msg->twist.linear.z;
    data.w_body_x = msg->twist.angular.x;
    data.w_body_y = msg->twist.angular.y;
    data.w_body_z = msg->twist.angular.z;

    data.velocity_body_received = true;

    // 如果所有信息都已接收到，写入 CSV
    if (data.position_received && data.velocity_body_received && data.velocity_local_received) {
        csv_file << data.pos_x << ", " << data.pos_y << ", " << data.pos_z << ", "
                 << data.qx << ", " << data.qy << ", " << data.qz << ", " << data.qw << ", "
                 << data.v_body_x << ", " << data.v_body_y << ", " << data.v_body_z << ", "
                 << data.w_body_x << ", " << data.w_body_y << ", " << data.w_body_z << ", "
                 << data.v_enu_x << ", " << data.v_enu_y << ", " << data.v_enu_z << ", "
                 << data.w_enu_x << ", " << data.w_enu_y << ", " << data.w_enu_z << std::endl;
        
        // 重置标记，准备接收下一组数据
        data.position_received = false;
        data.velocity_body_received = false;
        data.velocity_local_received = false;
    }
}

void localVelocityLocalCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // 提取ENU坐标系的速度信息
    data.v_enu_x = msg->twist.linear.x;
    data.v_enu_y = msg->twist.linear.y;
    data.v_enu_z = msg->twist.linear.z;
    data.w_enu_x = msg->twist.angular.x;
    data.w_enu_y = msg->twist.angular.y;
    data.w_enu_z = msg->twist.angular.z;

    data.velocity_local_received = true;

    // 如果所有信息都已接收到，写入 CSV
    if (data.position_received && data.velocity_body_received && data.velocity_local_received) {
        csv_file << data.pos_x << ", " << data.pos_y << ", " << data.pos_z << ", "
                 << data.qx << ", " << data.qy << ", " << data.qz << ", " << data.qw << ", "
                 << data.v_body_x << ", " << data.v_body_y << ", " << data.v_body_z << ", "
                 << data.w_body_x << ", " << data.w_body_y << ", " << data.w_body_z << ", "
                 << data.v_enu_x << ", " << data.v_enu_y << ", " << data.v_enu_z << ", "
                 << data.w_enu_x << ", " << data.w_enu_y << ", " << data.w_enu_z << std::endl;
        
        // 重置标记，准备接收下一组数据
        data.position_received = false;
        data.velocity_body_received = false;
        data.velocity_local_received = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "position_velocity_logger");
    ros::NodeHandle nh;

    // 打开 CSV 文件进行写入
    csv_file.open("/home/shuai/ros_boat_ws/boat_pose.csv", std::ios::out);
    if (!csv_file.is_open()) {
        ROS_ERROR("Unable to open CSV file.");
        return 1;
    }

    // 写入 CSV 文件的表头
    csv_file << "Pos_x, Pos_y, Pos_z, qx, qy, qz, qw, "
             << "V_body_x, V_body_y, V_body_z, W_body_x, W_body_y, W_body_z, "
             << "V_enu_x, V_enu_y, V_enu_z, W_enu_x, W_enu_y, W_enu_z" << std::endl;

    // 订阅相应的话题
    ros::Subscriber local_position_sub = nh.subscribe("/mavros/local_position/pose", 10, localPositionCallback);
    // 相对于机体坐标系的速度
    ros::Subscriber local_velocity_body_sub = nh.subscribe("/mavros/local_position/velocity_body", 10, localVelocityBodyCallback);
    // 相对于ENU坐标系的速度
    ros::Subscriber local_velocity_local_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, localVelocityLocalCallback);

    ros::spin();

    // 关闭文件
    csv_file.close();

    return 0;
}
