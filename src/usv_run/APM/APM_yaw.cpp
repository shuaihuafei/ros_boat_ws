#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <std_msgs/Float64.h>

// Global variables
mavros_msgs::State current_state;
double target_yaw = 0.0;  // Initial target yaw in radians

// Callback: Subscribe to the current state of the UAV
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// Callback: Subscribe to the target angle information
void objectAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    // Convert from degrees to radians
    target_yaw = msg->data * M_PI / 180.0;  // Assuming input is in degrees
    ROS_INFO("Received target angle: %f degrees, converted to %f radians", msg->data, target_yaw);
}

// Convert yaw (radians) to quaternion
geometry_msgs::Quaternion toQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Set only Yaw, Roll and Pitch set to 0
    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();
    return quat_msg;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "yaw_control_node");
    ros::NodeHandle nh;

    // Subscribe to UAV state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // Subscribe to target angle information
    ros::Subscriber yaw_sub = nh.subscribe<std_msgs::Float64>("/object_angle", 10, objectAngleCallback);

    // Publish target attitude (Yaw)
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);

    // Service client: Change flight mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // Service client: Arm the UAV
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // Wait for connection to the flight controller
    ros::Rate rate(10);
    while (ros::ok() && !current_state.connected) {
        ROS_INFO("Waiting for connection to the flight controller...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to the flight controller!");

    // Send initial target (must send a few times, otherwise mode switching and arming may fail)
    mavros_msgs::AttitudeTarget att_msg;
    att_msg.type_mask = 0b100111;  // Ignore Roll and Pitch
    att_msg.orientation = toQuaternion(target_yaw);  // Set initial target yaw
    att_msg.thrust = 0.5;  // Medium thrust
    for (int i = 0; ros::ok() && i < 100; ++i) {
        attitude_pub.publish(att_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // Switch mode to GUIDED
    mavros_msgs::SetMode set_mode_msg;
    set_mode_msg.request.custom_mode = "GUIDED";
    if (set_mode_client.call(set_mode_msg) && set_mode_msg.response.mode_sent) {
        ROS_INFO("Mode switched to GUIDED successfully");
    } else {
        ROS_ERROR("Failed to switch mode to GUIDED");
        return -1;
    }

    // Arm the UAV
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("UAV armed successfully");
    } else {
        ROS_ERROR("Failed to arm the UAV");
        return -1;
    }

    // Start loop to send dynamic target yaw
    ROS_INFO("Starting to send dynamic target yaw...");
    while (ros::ok()) {
        // Update target yaw with received angle
        att_msg.orientation = toQuaternion(target_yaw);
        attitude_pub.publish(att_msg);  // Publish target attitude
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
