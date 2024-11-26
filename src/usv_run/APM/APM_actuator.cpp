#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>

class ActuatorControlNode {
public:
    ActuatorControlNode(ros::NodeHandle& node_handle) 
        : nh(node_handle), control_value_2(0.2), control_value_3(0.6) {
        // 初始化订阅器、发布器和服务客户端
        state_sub = nh.subscribe("mavros/state", 10, &ActuatorControlNode::stateCb, this);
        actuator_control_pub = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 10);
        actuator_control_sub = nh.subscribe("/mavros/target_actuator_control", 10, &ActuatorControlNode::actuatorControlCallback, this);
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        // 从参数服务器读取控制信号值
        nh.param("control_value_2", control_value_2, 0.2);
        nh.param("control_value_3", control_value_3, 0.6);
        
        ROS_INFO("Control Value 2: %f, Control Value 3: %f", control_value_2, control_value_3);
    }

    // 启动节点，进行控制
    void run() {
        ros::Rate rate(20.0);

        // 等待飞行器连接
        while (ros::ok() && !current_state.connected) {
            ros::spinOnce();
            rate.sleep();
        }
        for(int i = 100; ros::ok() && i > 0; --i){
            sendActuatorControl();
            ros::spinOnce();
            rate.sleep();
        }

        // 等待飞行器准备好，进入 OFFBOARD 模式
        if (!setOffboardMode()) {
            ROS_ERROR("Failed to set mode to GUIDED");
            return;
        }

        // 解锁飞行器
        if (!armVehicle()) {
            ROS_ERROR("Failed to arm vehicle");
            return;
        }

        // 控制电机输出
        while (ros::ok()) {
            sendActuatorControl();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // ROS 相关变量
    ros::NodeHandle nh;
    ros::Subscriber state_sub;
    ros::Publisher actuator_control_pub;
    ros::Subscriber actuator_control_sub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    // 飞行器状态
    mavros_msgs::State current_state;

    // 控制信号值
    double control_value_2;
    double control_value_3;

    // 状态回调函数
    void stateCb(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    // 回调函数，当有新的 ActuatorControl 消息到来时被调用
    void actuatorControlCallback(const mavros_msgs::ActuatorControl::ConstPtr& msg)
    {
        // 打印控制组和控制信号（假设 controls 数组是电机输出的状态）
        ROS_INFO("Group Mix: %d", msg->group_mix);
        ROS_INFO("Controls:");
        for (size_t i = 0; i < msg->controls.size(); ++i)
        {
            ROS_INFO("Control %zu: %f", i, msg->controls[i]);
        }
    }

    // 发送电机控制信号
    void sendActuatorControl() {
        mavros_msgs::ActuatorControl actuator_control_msg;

        // 设置电机控制值
        actuator_control_msg.controls[0] = 0.0;
        actuator_control_msg.controls[1] = 0.0;
        actuator_control_msg.controls[2] = control_value_2;
        actuator_control_msg.controls[3] = control_value_3;
        actuator_control_msg.controls[4] = 0.0;
        actuator_control_msg.controls[5] = 0.0;
        actuator_control_msg.controls[6] = 0.0;
        actuator_control_msg.controls[7] = 0.0;

        actuator_control_msg.group_mix = 0;  // 控制组

        // 发布控制信号
        actuator_control_pub.publish(actuator_control_msg);
    }

    // 设置飞行器为 OFFBOARD 模式
    bool setOffboardMode() {
        mavros_msgs::SetMode set_mode;
        set_mode.request.base_mode = 0;
        set_mode.request.custom_mode = "GUIDED";  // 设置为 OFFBOARD 模式

        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
            ROS_INFO("Set mode to GUIDED");
            return true;
        }
        return false;
    }

    // 解锁飞行器
    bool armVehicle() {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;  // 解锁飞行器

        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
            return true;
        }
        return false;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "actuator_control_node");

    // 创建 ROS NodeHandle 实例
    ros::NodeHandle nh;

    ActuatorControlNode node(nh);  // 创建 ActuatorControlNode 对象
    node.run();  // 运行控制

    return 0;
}
