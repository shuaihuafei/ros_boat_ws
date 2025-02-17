#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>

class VelocityControlNode {
public:
    VelocityControlNode(ros::NodeHandle& nh)
        : nh_(nh), linear_speed_(0.5), angular_speed_(0.0) {
        // 订阅飞行器状态
        state_sub_ = nh_.subscribe("mavros/state", 10, &VelocityControlNode::stateCb, this);
        
        // 发布线速度指令
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 10);
    }

    // 启动控制
    void run() {
        ros::Rate rate(20.0);

        // 等待飞行器连接
        while (ros::ok() && !current_state_.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        // 控制飞行器运动
        while (ros::ok()) {
            sendVelocityCommand();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher velocity_pub_;
    
    mavros_msgs::State current_state_;
    
    // 控制参数：线速度和角速度
    double linear_speed_;
    double angular_speed_;

    // 飞行器状态回调
    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    // 发布线速度命令
    void sendVelocityCommand() {
        geometry_msgs::Twist cmd_vel_msg;

        // 设置线速度和角速度（假设是2D平面控制）
        cmd_vel_msg.linear.x = linear_speed_;  // x方向线速度
        cmd_vel_msg.linear.y = 0.0;            // y方向线速度（不动）
        cmd_vel_msg.linear.z = 0.0;            // z方向线速度（不动）
        
        cmd_vel_msg.angular.z = angular_speed_;  // 偏航角速度

        // 发布速度命令
        velocity_pub_.publish(cmd_vel_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_control_node");

    ros::NodeHandle nh;

    // 创建控制节点对象
    VelocityControlNode node(nh);

    // 启动控制
    node.run();

    return 0;
}
