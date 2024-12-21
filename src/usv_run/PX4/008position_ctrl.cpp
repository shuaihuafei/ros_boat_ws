#include <ros/ros.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <limits>
#include <geometry_msgs/Vector3.h>

class RoverPositionControl
{
public:
    RoverPositionControl()
        : _control_position_last_called(0), _control_mode_current(MODE_IDLE),
          _pos_ctrl_state(STATE_WAITING), _prev_wp(geometry_msgs::Point()), dt(0.01)
    {
        // 初始化 ROS 节点
        ros::NodeHandle nh;

        // 订阅 MAVROS 的位置和速度话题
        global_position_sub = nh.subscribe("/mavros/global_position/global", 10, &RoverPositionControl::globalPositionCallback, this);
        local_velocity_sub = nh.subscribe("/mavros/local_position/velocity_body", 10, &RoverPositionControl::localVelocityCallback, this);

        // 发布控制命令
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 10);

        // 参数初始化
        nh.param("param_wheel_base", _param_wheel_base, 1.0f);
        nh.param("param_max_turn_angle", _param_max_turn_angle, 45.0f);
    }

    void globalPositionCallback(const mavros_msgs::GlobalPositionTarget::ConstPtr& msg)
    {
        current_position.x = msg->latitude;
        current_position.y = msg->longitude;
        current_position.z = msg->altitude;
    }

    void localVelocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
    {
        ground_speed.x = msg->x;
        ground_speed.y = msg->y;
        ground_speed.z = msg->z;
    }

    bool controlPosition(const mavros_msgs::PositionTarget& pos_sp_triplet)
    {
        if (_control_position_last_called > 0) {
            dt = (ros::Time::now().toSec() - _control_position_last_called) * 1e-6f;
        }

        _control_position_last_called = ros::Time::now().toSec();

        bool setpoint = true;

        // 如果启用自动模式且目标有效
        if (_control_mode_current == MODE_AUTO && pos_sp_triplet.type_mask == 0)
        {
            // 获取当前位置和目标位置
            geometry_msgs::Point curr_wp;
            curr_wp.x = pos_sp_triplet.position.x;
            curr_wp.y = pos_sp_triplet.position.y;

            geometry_msgs::Point prev_wp = (pos_sp_triplet.position_valid) ? pos_sp_triplet.position : _prev_wp;

            geometry_msgs::Vector3 ground_speed_2d;
            ground_speed_2d.x = ground_speed.x;
            ground_speed_2d.y = ground_speed.y;

            // 设置油门
            float mission_throttle = controlThrottle(pos_sp_triplet);

            // 计算目标与当前位置的距离
            float dist_target = get_distance_to_next_waypoint(curr_wp.x, curr_wp.y);

            // 根据目标距离判断状态
            switch (_pos_ctrl_state)
            {
            case STATE_NAVIGATING:
                if (dist_target < _param_nav_loiter_rad)
                {
                    _pos_ctrl_state = STATE_STOPPING;
                }
                else
                {
                    navigateWaypoints(curr_wp, prev_wp, ground_speed_2d, mission_throttle);
                }
                break;

            case STATE_STOPPING:
                stopControl();
                break;

            default:
                ROS_ERROR("Unknown Rover State");
                _pos_ctrl_state = STATE_STOPPING;
                break;
            }

            _prev_wp = curr_wp;
        }
        else
        {
            _control_mode_current = MODE_IDLE;
            setpoint = false;
        }

        return setpoint;
    }

private:
    ros::Subscriber global_position_sub;
    ros::Subscriber local_velocity_sub;
    ros::Publisher cmd_vel_pub;

    geometry_msgs::Point current_position;
    geometry_msgs::Vector3 ground_speed;

    RoverControlMode _control_mode_current;
    RoverControlState _pos_ctrl_state;

    geometry_msgs::Point _prev_wp;

    float _control_position_last_called;
    float _param_wheel_base;
    float _param_max_turn_angle;

    float dt;  // Time step

    const float _param_nav_loiter_rad = 1.0f;  // Loiter radius for stopping

    // 控制油门
    float controlThrottle(const mavros_msgs::PositionTarget& pos_sp_triplet)
    {
        float mission_throttle = 0.5f; // 默认油门

        // 控制速度
        if (_param_speed_control_mode == 1)
        {
            float mission_target_speed = 1.0f;
            if (std::isfinite(pos_sp_triplet.cruising_speed) && pos_sp_triplet.cruising_speed > 0.1f)
            {
                mission_target_speed = pos_sp_triplet.cruising_speed;
            }

            // 计算地面速度
            float x_vel = ground_speed.x;
            float x_acc = 0.0f; // 可以设置为车载加速度传感器的值

            mission_throttle = 0.5f * pid_calculate(&_speed_ctrl, mission_target_speed, x_vel, x_acc, dt);
            mission_throttle = std::clamp(mission_throttle, 0.0f, 1.0f);
        }
        else
        {
            // 开环控制油门
            if (std::isfinite(pos_sp_triplet.cruising_throttle) && pos_sp_triplet.cruising_throttle > 0.01f)
            {
                mission_throttle = pos_sp_triplet.cruising_throttle;
            }
        }

        return mission_throttle;
    }

    // 控制航向
    void controlYaw(float control_effort)
    {
        geometry_msgs::Twist msg;
        msg.angular.z = control_effort;
        cmd_vel_pub.publish(msg);
    }

    // 控制油门
    void controlThrottle(float throttle)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = throttle;
        cmd_vel_pub.publish(msg);
    }

    // 停止控制
    void stopControl()
    {
        controlThrottle(0.0f);
        controlYaw(0.0f);
    }

    // 导航到下一个航点
    void navigateWaypoints(const geometry_msgs::Point& curr_wp, const geometry_msgs::Point& prev_wp, const geometry_msgs::Vector3& ground_speed_2d, float mission_throttle)
    {
        geometry_msgs::Point curr_pos_local;  // 假设当前位置
        geometry_msgs::Point curr_wp_local = curr_wp; // 假设项目参考系已经校准
        geometry_msgs::Point prev_wp_local = prev_wp;

        // 调用导航控制
        _gnd_control.navigate_waypoints(prev_wp_local, curr_wp_local, curr_pos_local, ground_speed_2d);

        // 设置油门
        controlThrottle(mission_throttle);

        // 计算期望转向角度
        float desired_r = std::sqrt(ground_speed_2d.x * ground_speed_2d.x + ground_speed_2d.y * ground_speed_2d.y) / std::fabs(_gnd_control.nav_lateral_acceleration_demand());
        float desired_theta = (0.5f * M_PI_F) - std::atan2(desired_r, _param_wheel_base);
        float control_effort = desired_theta / _param_max_turn_angle;
        controlYaw(control_effort);
    }

    // 计算当前位置到目标位置的距离
    float get_distance_to_next_waypoint(float lat, float lon)
    {
        // 可以使用 Haversine 公式计算距离
        return std::sqrt(std::pow(lat - current_position.x, 2) + std::pow(lon - current_position.y, 2));
    }
};
