#include <swarm_navigator/odom_listener.h>


#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


namespace swarm_navigator {

    /*    
        OdomListener implementations    
    */

    OdomListener::OdomListener(ros::NodeHandle &nh,std::string topic){
        nh_ = nh;   
        topic_ = topic;

        seq_ = 0;
        position_ = Position();  
        orientation_ = Orientation(); 
        linear_velocity_ = 0.0;
        angular_velocity_ = 0.0;
    }

    Position OdomListener::getPosition(){
       return position_;
    }

    Orientation OdomListener::getOrientation(){
       return orientation_;
    }

    double OdomListener::getLinerVelocity(){
        return linear_velocity_;
    }

    double OdomListener::getAngularVelocity(){
        return angular_velocity_;
    }

    void OdomListener::start(){
        // subscriber = nh_.subscribe(topic_, 1000, update);
        subscriber = nh_.subscribe(topic_, 1000, &OdomListener::update, this);
        ros::spin();
    }

    void OdomListener::update(const nav_msgs::Odometry::ConstPtr& msg){
        // seq_ = msg->header.seq;
        // position_.update( msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
        // orientation_.update(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        // linear_velocity_ = msg->twist.twist.linear.x;
        // angular_velocity_ = msg->twist.twist.angular.z;

        // ROS_INFO("Seq: [%d]", seq_);
        // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", position_.getX(),position_.getY(), position_.getZ());
        // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]",orientation_.getX(), orientation_.getY(), orientation_.getZ(), orientation_.getW());
        // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", linear_velocity_,angular_velocity_);

        ROS_INFO("odom recieved ######################");
        

    }

    /*    
        Position implementations    
    */

        void Position::setX(double x){
            x_ = x;
        }
        void Position::setY(double y){
            y_ = y;
        }
        void Position::setZ(double z){
            z_ = z;
        }
        
        double Position::getX(){
            return x_;
        }
        double Position::getY(){
            return y_;
        }
        double Position::getZ(){
            return z_;
        }

        void Position::update(double x,double y,double z){
            x_ = x;
            y_ = y;
            z_ = z;
        }

    /*    
        Orientation implementations    
    */

        void Orientation::setX(double x){
            x_ = x;
        }
        void Orientation::setY(double y){
            y_ = y;
        }
        void Orientation::setZ(double z){
            z_ = z;
        }
        void Orientation::setW(double w){
            w_ = w;
        }
        
        double Orientation::getX(){
            return x_;
        }
        double Orientation::getY(){
            return y_;
        }
        double Orientation::getZ(){
            return z_;
        }
         double Orientation::getW(){
            return w_;
        }

        void Orientation::update(double x,double y,double z,double w){
            x_ = x;
            y_ = y;
            z_ = z;
            w_ = w;
        }
}


// void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
// {
//   ROS_INFO("Seq: [%d]", msg->header.seq);
//   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
//   ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//   ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
// }

// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "odom_listener");

//   ros::NodeHandle n;

//   ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  
//   ros::spin();

//   return 0;
// }