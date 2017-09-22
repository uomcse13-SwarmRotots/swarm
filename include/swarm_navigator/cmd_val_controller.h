#ifndef CMD_VAL_CONTROLLER_INCLUDE
#define CMD_VAL_CONTROLLER_INCLUDE

#include <string> 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <math.h>


namespace swarm_navigator {

class CmdValController{

    private:
    
        ros::NodeHandle nh_;  
        ros::Publisher publisher_;
        tf::TransformListener listener_;
        std::string base_link_;
        std::string odom_link_;

        float liner_velocity_;
        float angular_velocity_;

        bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;
        

    public:
    
        CmdValController(ros::NodeHandle &nh,std::string topic);       
        CmdValController(ros::NodeHandle &nh,std::string topic,std::string base_link,std::string odom_link);
        void stop();
        void foward();
        void backward();
        void left();
        void right();
       
        
        bool driveForward(double distance);
        bool turn(bool clockwise, double radians);
        bool achieveGoal(const geometry_msgs::PoseStamped& goal);

        void setLinerVelocity(float velocity);
        void setAngularVelocity(float velocity);
        bool ready();

};


}

#endif