/*
******************************************************************************
*       @function       get_local_pose
*       @author         Li Jiye
*       @brief          将gazebo中的真值发送给无人机
*       @date           2024/1//31
*******************************************************************************
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <gazebo_msgs/ModelStates.h>

ros::Subscriber gazebo_model_state_sub;
ros::Publisher pose_pub;
ros::Publisher speed_pub;

geometry_msgs::PoseStamped localPose;
geometry_msgs::Vector3Stamped speed;
bool found = false;
std::string model_name;

void gazebo_model_state_callback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        if(msg->name[i] == "iris_0")
        {
            found = true;
            std::cout << "Find it" << std::endl;
            localPose.header.stamp = ros::Time::now();
            localPose.header.frame_id = "map";
            localPose.pose.position.x = msg->pose[i].position.x;
            localPose.pose.position.y = msg->pose[i].position.y;
            localPose.pose.position.z = msg->pose[i].position.z;

            localPose.pose.orientation.w = msg->pose[i].orientation.w;
            localPose.pose.orientation.x = msg->pose[i].orientation.x;
            localPose.pose.orientation.y = msg->pose[i].orientation.y;
            localPose.pose.orientation.z = msg->pose[i].orientation.z;            

            speed.header.stamp = ros::Time::now();
            speed.header.frame_id = "map";
            speed.vector.x = msg->twist[i].linear.x;    
            speed.vector.y = msg->twist[i].linear.y; 
            speed.vector.z = msg->twist[i].linear.z; 

            pose_pub.publish(localPose);
            speed_pub.publish(speed);    
        }        
        if (!found) {
            ROS_WARN("Model 'iris' not found in Gazebo!");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_local_pose");
    ros::NodeHandle nh("~");


    nh.param<std::string>("model_name", model_name, "iris_0"); // 默认值为 "iris_0"
    gazebo_model_state_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 100, gazebo_model_state_callback);
    
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/iris_0/mavros/vision_pose/pose", 10);
    speed_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/iris_0/mavros/vision_speed/speed", 10);

    ros::Rate rate(20);

    while(ros::ok())
    {

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
