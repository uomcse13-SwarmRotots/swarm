#ifndef ODOM_LISTENER_INCLUDE
#define ODOM_LISTENER_INCLUDE

#include <string> 
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace swarm_navigator {

class Position {
    private:

        double x_,y_,z_;       

    public:

        Position(){};

        void setX(double x);
        void setY(double y);
        void setZ(double z);
        
        double getX();
        double getY();
        double getZ();

        void update(double x,double y,double z);
        
};

class Orientation {
    private:

        double x_,y_,z_,w_;

    public:

        Orientation(){};

        void setX(double x);
        void setY(double y);
        void setZ(double z);
        void setW(double w);

        double getX();
        double getY();
        double getZ();
        double getW();

        void update(double x,double y,double z,double w);
        
};

class OdomListener{

    private:
    
        ros::NodeHandle nh_;  
        ros::Subscriber subscriber;
        std::string topic_;
        

        int seq_;
        Position position_;
        Orientation orientation_;
        double linear_velocity_;
        double angular_velocity_;
        

        void update(const nav_msgs::Odometry::ConstPtr& msg);
       

    public:
    
        OdomListener(ros::NodeHandle &nh,std::string topic);       

        int getSeq();
        Position getPosition();
        Orientation getOrientation();
        double getLinerVelocity();
        double getAngularVelocity();

        void start();

};

}

#endif

//   ROS_INFO("Seq: [%d]", msg->header.seq);
//   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//   ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//   ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
// }