#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <cmath>

class PX4GlobalPositionController {
private:
    ros::NodeHandle nh_;
    
    // 订阅者
    ros::Subscriber state_sub_;
    ros::Subscriber global_pos_sub_;
    ros::Subscriber target_pos_sub_;    // 现在使用NavSatFix类型
    
    // 发布者
    ros::Publisher global_pos_pub_;
    
    // 服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    
    // 消息存储
    mavros_msgs::State current_state_;
    sensor_msgs::NavSatFix current_global_pos_;
    sensor_msgs::NavSatFix target_pos_;         // 存储目标经纬度
    geographic_msgs::GeoPoseStamped setpoint_;  // 用于发布给MAVROS的消息
    bool target_received_;
    
    // 参数
    double position_tolerance_;
    ros::Rate* rate_;
    
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    
    void globalPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        current_global_pos_ = *msg;
    }

    // 简化后的目标位置回调函数
    void targetPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        target_pos_ = *msg;
        
        // 更新setpoint消息
        setpoint_.header.stamp = ros::Time::now();
        setpoint_.header.frame_id = "map";
        setpoint_.pose.position.latitude = msg->latitude;
        setpoint_.pose.position.longitude = msg->longitude;
        setpoint_.pose.position.altitude = msg->altitude;
        
        // 保持默认朝向
        setpoint_.pose.orientation.x = 0.0;
        setpoint_.pose.orientation.y = 0.0;
        setpoint_.pose.orientation.z = 0.0;
        setpoint_.pose.orientation.w = 1.0;
        
        target_received_ = true;
        ROS_INFO("Received new target position: Lat: %.6f, Lon: %.6f, Alt: %.2f",
                 msg->latitude, msg->longitude, msg->altitude);
    }
    
    double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000;
        
        double lat1_rad = lat1 * M_PI / 180.0;
        double lon1_rad = lon1 * M_PI / 180.0;
        double lat2_rad = lat2 * M_PI / 180.0;
        double lon2_rad = lon2 * M_PI / 180.0;
        
        double dlat = lat2_rad - lat1_rad;
        double dlon = lon2_rad - lon1_rad;
        double a = sin(dlat/2) * sin(dlat/2) +
                  cos(lat1_rad) * cos(lat2_rad) *
                  sin(dlon/2) * sin(dlon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return R * c;
    }

public:
    PX4GlobalPositionController() : nh_("~"), target_received_(false) {
        // 初始化订阅者
        state_sub_ = nh_.subscribe<mavros_msgs::State>
                ("/mavros/state", 10, &PX4GlobalPositionController::stateCallback, this);
        global_pos_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>
                ("/mavros/global_position/global", 10, &PX4GlobalPositionController::globalPosCallback, this);
        target_pos_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>
                ("/target_global_position", 10, &PX4GlobalPositionController::targetPosCallback, this);
        
        // 初始化发布者
        global_pos_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>
                ("/mavros/setpoint_position/global", 10);
        
        // 初始化服务客户端
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
                ("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
                ("/mavros/set_mode");
        
        // 初始化参数
        position_tolerance_ = 2.0;
        rate_ = new ros::Rate(20.0);
    }
    
    ~PX4GlobalPositionController() {
        delete rate_;
    }
    
    bool reachedTarget() {
        if (!target_received_) return false;
        
        double distance = calculateDistance(
            current_global_pos_.latitude,
            current_global_pos_.longitude,
            target_pos_.latitude,
            target_pos_.longitude
        );
        return distance < position_tolerance_;
    }
    
    void run() {
        // 等待连接
        while(ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate_->sleep();
        }
        
        // 等待接收到第一个目标位置
        ROS_INFO("Waiting for target position...");
        while(ros::ok() && !target_received_) {
            ros::spinOnce();
            rate_->sleep();
        }
        
        // 发送几个空命令
        for(int i = 100; ros::ok() && i > 0; --i) {
            global_pos_pub_.publish(setpoint_);
            ros::spinOnce();
            rate_->sleep();
        }
        
        // 切换到OFFBOARD模式并尝试解锁
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        ros::Time last_request = ros::Time::now();
        
        while(ros::ok()) {
            if(current_state_.mode != "OFFBOARD" &&
               (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if(set_mode_client_.call(offb_set_mode) &&
                   offb_set_mode.response.mode_sent) {
                    ROS_INFO("OFFBOARD enabled");
                }
                last_request = ros::Time::now();
            } else {
                if(!current_state_.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if(arming_client_.call(arm_cmd) &&
                       arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            
            // 发布位置命令
            global_pos_pub_.publish(setpoint_);
            
            // 检查是否到达目标位置
            if(reachedTarget()) {
                ROS_INFO("Reached target position");
            }
            
            ros::spinOnce();
            rate_->sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "px4_global_position_controller");
    
    try {
        PX4GlobalPositionController controller;
        controller.run();
    }
    catch(const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}