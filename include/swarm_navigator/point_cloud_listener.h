#ifndef POINT_CLOUD_LISTENER_INCLUDE
#define POINT_CLOUD_LISTENER_INCLUDE

#include <string> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace swarm_navigator {

class PointCloudListener{

    private:
    
        ros::NodeHandle nh_;  
        ros::Subscriber subscriber;
        std::string topic_;
          

        void update(const sensor_msgs::PointCloud2ConstPtr& msg);
       

    public:
    
        PointCloudListener(ros::NodeHandle &nh,std::string topic);       

        void start();

};

}

#endif
