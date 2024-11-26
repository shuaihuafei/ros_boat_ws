#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float32MultiArray.h>
#include <mavros_msgs/CommandBool.h>

class OffboardControlNode
{
public:
    OffboardControlNode()
    {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 发布遥控器输入和解锁命令
        rc_in_pub = nh.advertise<mavros_msgs::RCIn>("mavros/rc/in", 10);
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        arm_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    }

    // 解锁飞控
    bool arm()
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true; // 解锁
        if (arm_client.call(arm_cmd) && arm_cmd.response.success)
        {
            ROS_INFO("Armed successfully!");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to arm!");
            return false;
        }
    }

    // 切换到 Offboard 模式
    bool setOffboardMode()
    {
        mavros_msgs::SetMode mode_cmd;
        mode_cmd.request.custom_mode = "OFFBOARD"; // 设置为 Offboard 模式
        if (set_mode_client.call(mode_cmd) && mode_cmd.response.mode_sent)
        {
            ROS_INFO("Offboard mode activated!");
            return true;
        }
        else
        {
            ROS_ERROR("Failed to set Offboard mode!");
            return false;
        }
    }

    // 模拟遥控器输入
    void sendRCControl(float motor1_thrust, float motor2_thrust)
    {
        mavros_msgs::RCIn rc_msg;

        // 使用 std::min 和 std::max 限制电机推力值在 1000 到 2000 之间
        rc_msg.channels[0] = std::min(std::max(static_cast<int>(motor1_thrust), 1000), 2000);  // 电机 1 推力
        rc_msg.channels[1] = std::min(std::max(static_cast<int>(motor2_thrust), 1000), 2000);  // 电机 2 推力

        // 发布遥控器信号
        rc_in_pub.publish(rc_msg);
    }

private:
    ros::Publisher rc_in_pub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arm_client;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "offboard_control_node");
    OffboardControlNode offboard_control_node;

    ros::Rate rate(10); // 控制频率，10Hz

    // 等待 MAVROS 和飞控启动
    ros::Duration(2).sleep();

    // 解锁飞控
    if (!offboard_control_node.arm())
    {
        ROS_ERROR("Failed to arm the vehicle.");
        return 1;
    }

    // 切换到 Offboard 模式
    if (!offboard_control_node.setOffboardMode())
    {
        ROS_ERROR("Failed to set Offboard mode.");
        return 1;
    }

    while (ros::ok())
    {
        // 模拟遥控器输入（电机推力值示例）
        offboard_control_node.sendRCControl(1500, 1600);  // 控制信号值

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
