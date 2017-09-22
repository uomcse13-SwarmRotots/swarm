#include <swarm_navigator/point_cloud_listener.h>


#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>


namespace swarm_navigator {

    /*    
        PointCloudListener implementations    
    */

    PointCloudListener::PointCloudListener(ros::NodeHandle &nh,std::string topic){
        nh_ = nh;   
        topic_ = topic;
    }

    void PointCloudListener::start(){
        // subscriber = nh_.subscribe(topic_, 1000, update);
        subscriber = nh_.subscribe(topic_, 1000, &PointCloudListener::update, this);
        ros::spin();
    }

    void PointCloudListener::update(const sensor_msgs::PointCloud2ConstPtr& msg){
        // seq_ = msg->header.seq;
        // position_.update( msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
        // orientation_.update(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        // linear_velocity_ = msg->twist.twist.linear.x;
        // angular_velocity_ = msg->twist.twist.angular.z;

        // ROS_INFO("Seq: [%d]", seq_);
        // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", position_.getX(),position_.getY(), position_.getZ());
        // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",orientation_.getX(), orientation_.getY(), orientation_.getZ(), orientation_.getW());
        // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", linear_velocity_,angular_velocity_);

        ROS_INFO("pointcloud recieved______________________");

    }


}
