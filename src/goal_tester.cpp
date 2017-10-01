#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include <swarm_navigator/cmd_val_controller.h>
#include <swarm_navigator/odom_listener.h>
#include <swarm_navigator/point_cloud_listener.h>

using namespace sensor_msgs;
using namespace message_filters;

swarm_navigator::CmdValController* controller;

// void callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& points)
// {
//     // if(odom==NULL)
//     //     ROS_INFO("pointcloud recieved______________________");
//     // else
//     //     ROS_INFO("odom recieved ######################");

//     ROS_INFO("ODOM Seq: [%d]", odom->header.seq);
//     ROS_INFO("POINT Seq: [%d]", points->header.seq);
    
// }

void callback(const geometry_msgs::PoseStamped& goal)
{
    // if(odom==NULL)
    //     ROS_INFO("pointcloud recieved______________________");
    // else
    //     ROS_INFO("odom recieved ######################");

    ROS_INFO("Came....");
    // ROS_INFO("POINT Seq: [%d]", points->header.seq);

   
    controller->achieveGoal(goal);
   
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");

    ros::NodeHandle nh;
    std::string topic_cmd_val;
    std::string topic_odom;
    std::string base_link;
    std::string odom_link;

    ros::NodeHandle private_nh("~");     
    private_nh.param("cmd_vel_topic", topic_cmd_val, std::string("/mobile_base/commands/velocity")); 
    private_nh.param("odom_topic", topic_odom, std::string("/odom")); 
    private_nh.param("base_link", base_link, std::string("base_footprint")); 
    private_nh.param("odom_link", odom_link, std::string("odom")); 

    controller = new swarm_navigator::CmdValController(nh,topic_cmd_val,base_link,odom_link);
    
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);
    // message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/depth_camera/depth_camera/depth/points", 1);
    // TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync(odom_sub, points_sub, 10);
    // sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::NodeHandle simple_nh("move_base_simple");
    ros::Subscriber goal_sub_ = simple_nh.subscribe("goal", 1, callback);
    // ros::Subscriber goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1);
    ros::spin();

    return 0;
}