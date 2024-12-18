#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>

class ActuatorControlSubscriber
{
public:
    ActuatorControlSubscriber()
    {
        // 初始化 ROS 节点句柄
        ros::NodeHandle nh;

        // 订阅 ActuatorControl 消息
        actuator_control_sub_ = nh.subscribe("/mavros/target_actuator_control", 10, &ActuatorControlSubscriber::actuatorControlCallback, this);
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

private:
    ros::Subscriber actuator_control_sub_;
};

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "actuator_control_listener");

    // 创建 ActuatorControlSubscriber 对象
    ActuatorControlSubscriber actuator_control_subscriber;

    // 进入 ROS 循环，等待消息
    ros::spin();

    return 0;
}
