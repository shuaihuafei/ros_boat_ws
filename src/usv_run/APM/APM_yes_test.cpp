#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>   
#include <string>

using namespace std;

//无人机GPS初始位置，需要通过此处校准，去除GPS上电后的漂移误差
float  init_position_x,init_position_y,init_position_z;
//初始化位置校准标志位
int    flag_init_position = 0;

mavros_msgs::State current_state;  

nav_msgs::Odometry local_pos;

mavros_msgs::PositionTarget setpoint_pos_raw;
mavros_msgs::AttitudeTarget setpoint_att_raw;

//ros::Time last_request = ros::Time::now();

void state_cb(const mavros_msgs::State::ConstPtr& msg);

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg);



int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_throw");

    ros::NodeHandle nh;
	 
	//订阅无人机状态话题
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
		
	//订阅无人机实时位置信息
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    
	//发布无人机位置控制话题
	ros::Publisher  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
		
	//发布无人机多维控制话题
    ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);   
    ros::Publisher  mavros_setpoint_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
		               
	//请求无人机解锁服务        
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		
	//请求无人机设置飞行模式，本代码请求进入offboard
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//请求控制舵机客户端
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    //循环频率
    ros::Rate rate(20.0); 
   
    //等待连接到PX4无人机
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }


       //机体坐标系下发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
	   // setpoint_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
		//setpoint_raw.coordinate_frame = 8;
		//setpoint_raw.velocity.x = 0.2;
		//setpoint_raw.velocity.y = 0.2;
		//setpoint_raw.yaw = -1.57;
		//yaw_increase = yaw_increase + 0.0001;
		//setpoint_raw.yaw_rate = 0.5;

		//setpoint_raw.position.z = init_position_z;//



	//setpoint_raw.position.z = init_position_z;
	mavros_setpoint_att_pub.publish(setpoint_att_raw);
 
    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_att_pub.publish(setpoint_att_raw);
        ros::spinOnce();
        rate.sleep();
    }

    //请求offboard模式变量
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";
 
    //请求解锁变量
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
   
    ros::Time last_request = ros::Time::now();
    int flag=0;
    //请求进入offboard模式并且解锁无人机，15秒后退出，防止重复请求       
    while(ros::ok())
    {
    	// printf("11111111111\r\n");
    	//请求进入OFFBOARD模式
        if( current_state.mode != "GUIDED" && (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("GUIDED enabled");
            }
            else
            {
               ROS_INFO("GUIDED failed");
            }
            
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
				//break;
		        }
                else
                {
                    ROS_INFO("arm failed");
                }
		        	last_request = ros::Time::now();
			}
		}
                  if(ros::Time::now() - last_request > ros::Duration(10.0))
        {
            flag++;
            last_request = ros::Time::now();
        }
        if(flag==0)
        {
	    setpoint_pos_raw.type_mask = /*1 + 2 + 4 + */8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_pos_raw.coordinate_frame = 1;
		setpoint_pos_raw.position.x = 10;
		setpoint_pos_raw.position.y =10;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("pos localned");
        }
        if(flag==1)
        {
	    setpoint_pos_raw.type_mask =1 + 2 + 4 + 8 + 16 + 32 + 64 + 128 + 256 + /*512 + 1024 + */2048;
		setpoint_pos_raw.coordinate_frame = 1;
		setpoint_pos_raw.yaw=3.14;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("yaw localned");
        }
        if(flag==2)
        {
	    setpoint_pos_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_pos_raw.coordinate_frame = 1;
		setpoint_pos_raw.velocity.x = 1;
		setpoint_pos_raw.velocity.y = 1;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("velocity localned");
        }
        if(flag==3)
        {
	    setpoint_pos_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_pos_raw.coordinate_frame = 8;
		setpoint_pos_raw.velocity.x = 1;
		setpoint_pos_raw.velocity.y = 0;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("velocity bodyned");
        }
        if(flag==4)
        {
	    setpoint_pos_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
		setpoint_pos_raw.coordinate_frame = 8;
		setpoint_pos_raw.velocity.x = 1;
		setpoint_pos_raw.velocity.y = 0;
        setpoint_pos_raw.yaw_rate=1;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("velocity bodyned and yawrate");
        }
        if(flag==5)
        {
            setpoint_att_raw.type_mask =163;
	        setpoint_att_raw.body_rate.z = 0.5;
	        setpoint_att_raw.thrust      = 1.0;
		    mavros_setpoint_att_pub.publish(setpoint_att_raw);
            ROS_INFO("thr and yawrate");
        }
        if(flag==6)
        {
            setpoint_att_raw.type_mask =39;
	        setpoint_att_raw.orientation.x = 1.0;
            setpoint_att_raw.orientation.y = 0;
            setpoint_att_raw.orientation.z = 0;
            setpoint_att_raw.orientation.w = 0;
	        setpoint_att_raw.thrust      = 1.0;
		    mavros_setpoint_att_pub.publish(setpoint_att_raw);
            ROS_INFO("thr and att");
        }

        ros::spinOnce();
        rate.sleep();
    }   
    return 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    //任意高度x或者x，y位置不为0，则表示已经接收到定点位置信息，记录第一次位置作为初始位置，防止上电时刻漂移过大
    if (flag_init_position==0 && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x = local_pos.pose.pose.position.x;
	    init_position_y = local_pos.pose.pose.position.y;
	    init_position_z = local_pos.pose.pose.position.z;
        flag_init_position = 1;		    
    }
}


