/**
 * @file ego_to_mavros.cpp
 * @brief 接收ego发出的目标点，并发送给mavros
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <offboard/PositionCommand.h>
#include <Eigen/Eigen>

int state_flag = 0;                 // 状态标志位
int ego_poscmd_count = 0;

// 接收飞控模式
mavros_msgs::State currentState;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    currentState = *msg;
}

Eigen::Vector3d currentPose;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPose(0) = msg->pose.position.x;
    currentPose(1) = msg->pose.position.y;
    currentPose(2) = msg->pose.position.z;
}

Eigen::Vector3d ego_poscmd;
double yaw_set;
void ego_pos_cb(const offboard::PositionCommand::ConstPtr& msg)
{
    ego_poscmd[0] = msg->position.x;
    ego_poscmd[1] = msg->position.y;
    ego_poscmd[2] = msg->position.z;
    yaw_set = msg->yaw;
    
    ego_poscmd_count = ego_poscmd_count + 1;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ego_to_mavros");
    ros::NodeHandle nh("~");

    float TAKEOFF_HEIGHT = 1.5;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/iris_0/mavros/state", 10, state_cb);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/iris_0/mavros/local_position/pose", 10, local_position_cb);
    ros::Subscriber ego_pos_sub = nh.subscribe<offboard::PositionCommand>("/planning/pos_cmd", 10, ego_pos_cb);

    ros::Publisher setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 10);

    ros::Rate rate(20.0);
    while(ros::ok() && !currentState.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_WARN("Waiting for connect");
    }
    ROS_INFO("Connected Success!");
    ros::Time last_request = ros::Time::now();

    mavros_msgs::PositionTarget setpoint_raw_local;
    setpoint_raw_local.type_mask = 0b100111111000;
    setpoint_raw_local.coordinate_frame = 1;
    setpoint_raw_local.position.x = 0;
    setpoint_raw_local.position.y = 0;
    setpoint_raw_local.position.z = TAKEOFF_HEIGHT;
    setpoint_raw_local.yaw = 0;
    while(ros::ok())
    {
        if(currentState.mode == "OFFBOARD")
        {
            // std::cout << "ego_poscmd:" << ego_poscmd.transpose() << std::endl;
            // ROS_INFO("Offboard Success");
            if(abs(currentPose(0) < 0.1) && abs(currentPose(1) < 0.1) && (abs(currentPose(2) - TAKEOFF_HEIGHT) <  0.1) &&
                    ego_poscmd_count > 1)
            {
                state_flag = 1;
            }
            switch(state_flag)
            {
                case 0:
                    setpoint_raw_local.type_mask = 0b100111111000;
                    setpoint_raw_local.coordinate_frame = 1;
                    setpoint_raw_local.position.x = 0;
                    setpoint_raw_local.position.y = 0;
                    setpoint_raw_local.position.z = TAKEOFF_HEIGHT;
                    setpoint_raw_local.yaw = 0; 
                    break;
                case 1:             // 飞到目标点之后，ego_planner也会不断发送位置信息
                    setpoint_raw_local.type_mask = 0b100111111000;
                    setpoint_raw_local.coordinate_frame = 1;
                    setpoint_raw_local.position.x = ego_poscmd(0);
                    setpoint_raw_local.position.y = ego_poscmd(1);
                    setpoint_raw_local.position.z = ego_poscmd(2);
                    setpoint_raw_local.yaw = yaw_set; 
                    break;
                default:
                    ROS_WARN("Error");
                    break;
            }
        }
        else
        {
            // ROS_WARN("Waiting for Offboard");
        }
        setpoint_raw_local_pub.publish(setpoint_raw_local);

        ros::spinOnce();
        rate.sleep();
    }    
    return 0;
}
