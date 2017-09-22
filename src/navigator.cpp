#include <iostream>
#include <swarm_navigator/cmd_val_controller.h>
#include <swarm_navigator/odom_listener.h>
#include <swarm_navigator/point_cloud_listener.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace swarm_navigator {

class Navigator{

    private:
    ros::NodeHandle nh_;

    float liner_velocity;
    float angular_velocity;

    public:

    
    Navigator(ros::NodeHandle &nh){
        nh_ = nh;
        liner_velocity = 0.25;
        angular_velocity = 0.75;

    }

    void executeContoller(std::string topic){
        CmdValController* controller = new CmdValController(nh_,topic);
        //   controller.driveKeyboard();
        double d;
        

        char cmd[50];

        while(controller->ready()){       
        

            std::cin.getline(cmd, 50);
            // if(cmd[0]!='w' && cmd[0]!='a' && cmd[0]!='d' && cmd[0]!='s' && cmd[0]!='q')
            // {
            //     std::cout << "invalid command:" << cmd << "\n";
            //     continue;
            // }
        
            if(cmd[0]=='w'){
                controller->foward();
            } 
            //turn left 
            else if(cmd[0]=='a'){
                controller->left();
            } 
            //turn right 
            else if(cmd[0]=='d'){
                controller->right();
            } 
            //turn back 
            else if(cmd[0]=='s'){
                controller->backward();
            } 
            //turn stop 
            else if(cmd[0]=='x'){
                controller->stop();
            } 
            else if(cmd[0]=='g'){
              std::cin >> d;
              controller->driveForward(d);
            }
            else if(cmd[0]=='l'){
              std::cin >> d;
              controller->turn(false,d);
            } 
            else if(cmd[0]=='r'){
              std::cin >> d;
              controller->turn(true,d);
            } 
            //quit
            else if(cmd[0]=='q'){
                break;
            }
        }

    }

    void executeOdomListener(std::string topic){
         OdomListener* o_listener = new OdomListener(nh_,topic);
         o_listener->start();
    }

    void executePointCloudListener(std::string topic){
        PointCloudListener* p_listener = new PointCloudListener(nh_,topic);
        p_listener->start();
    }


};

}

int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "navigator");
    ros::NodeHandle nh;
    std::string topic_cmd_val;
    std::string topic_odom;
    std::string topic_point_cloud;

    ros::NodeHandle private_nh("~");     
    private_nh.param("cmd_vel_topic", topic_cmd_val, std::string("/cmd_vel")); 
    private_nh.param("odom_topic", topic_odom, std::string("/odom")); 
    private_nh.param("point_cloud_topic", topic_point_cloud, std::string("/depth_camera/depth_camera/depth/points")); 
    
        

    swarm_navigator::Navigator* navigator = new swarm_navigator::Navigator(nh);
    navigator->executeContoller(topic_cmd_val);
    // navigator->executePointCloudListener(topic_point_cloud);    
    // navigator->executeOdomListener(topic_odom);
   

      
    

}

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <nav_msgs/Odometry.h>

// using namespace sensor_msgs;
// using namespace message_filters;

// void callback(const nav_msgs::Odometry::ConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& points)
// {
//     // if(odom==NULL)
//     //     ROS_INFO("pointcloud recieved______________________");
//     // else
//     //     ROS_INFO("odom recieved ######################");

//     ROS_INFO("ODOM Seq: [%d]", odom->header.seq);
//     ROS_INFO("POINT Seq: [%d]", points->header.seq);
    
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "navigation");

//   ros::NodeHandle nh;

//   message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);
//   message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub(nh, "/depth_camera/depth_camera/depth/points", 1);
//   TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2> sync(odom_sub, points_sub, 10);
//   sync.registerCallback(boost::bind(&callback, _1, _2));

//   ros::spin();

//   return 0;
// }