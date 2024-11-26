#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>

// Global variables: current state of the boat and target yaw angle
mavros_msgs::State current_state;
float target_yaw_angle = 0.0;  // Target yaw angle
ros::Time last_detection_time; // Time of last target detection

// State callback function
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// Angle information callback function
void objectAngleCallback(const std_msgs::Float64::ConstPtr& msg) {
    target_yaw_angle = msg->data;  // Get target yaw angle
    last_detection_time = ros::Time::now();  // Update detection time
    ROS_INFO("Received target yaw angle: %f radians", target_yaw_angle);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "boat_run_with_yaw_control_pid");
    ros::NodeHandle nh;

    // Subscribe to the current state of the boat
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

    // Subscribe to target angle information
    ros::Subscriber yaw_sub = nh.subscribe("/object_angle", 10, objectAngleCallback);

    // Create publisher for attitude control commands
    ros::Publisher cmd_attitude_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude", 10);

    // Create service clients for arming and setting mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Wait for ROS and flight controller to connect
    ros::Rate rate(20);  // 20 Hz publishing frequency
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Boat connected.");

    // Prepare requests for arming and mode setting
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD"; 

    // Record request time
    ros::Time last_request = ros::Time::now();

    // Initialize attitude control message
    geometry_msgs::PoseStamped cmd_attitude_msg;
    cmd_attitude_msg.pose.position.x = 0;
    cmd_attitude_msg.pose.position.y = 0;
    cmd_attitude_msg.pose.position.z = 0;

    bool armed = false;
    bool offboard_mode = false;

    // Publish initial attitude setpoint for 3 seconds before switching to OFFBOARD mode
    ros::Time start_time = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 3.0) {
        tf2::Quaternion q;
        q.setRPY(0, 0, target_yaw_angle);  // Set yaw angle
        cmd_attitude_msg.pose.orientation.x = q.x();
        cmd_attitude_msg.pose.orientation.y = q.y();
        cmd_attitude_msg.pose.orientation.z = q.z();
        cmd_attitude_msg.pose.orientation.w = q.w();

        cmd_attitude_pub.publish(cmd_attitude_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Published initial setpoint for 3 seconds.");

    // Control loop
    while (ros::ok()) {
        ros::spinOnce();

        // Check if more than 10 seconds have passed without detecting a target
        if ((ros::Time::now() - last_detection_time).toSec() > 10.0) {
            target_yaw_angle = 0.0;  // Stop turning
            ROS_WARN("No target detected for 10 seconds, setting yaw to 0.");
        }

        // Convert target yaw angle to quaternion
        tf2::Quaternion q;
        q.setRPY(0, 0, target_yaw_angle);
        cmd_attitude_msg.pose.orientation.x = q.x();
        cmd_attitude_msg.pose.orientation.y = q.y();
        cmd_attitude_msg.pose.orientation.z = q.z();
        cmd_attitude_msg.pose.orientation.w = q.w();

        // Continuously publish attitude control command
        cmd_attitude_pub.publish(cmd_attitude_msg);

        // Attempt to arm
        if (!armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Boat armed successfully.");
                armed = true;
            } else {
                ROS_ERROR("Failed to arm the boat.");
            }
            last_request = ros::Time::now();
        }

        // After successfully arming, switch to OFFBOARD mode
        if (armed && !offboard_mode && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent) {
                ROS_INFO("OFFBOARD mode set successfully.");
                offboard_mode = true;
            } else {
                ROS_ERROR("Failed to set OFFBOARD mode.");
            }
            last_request = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}
