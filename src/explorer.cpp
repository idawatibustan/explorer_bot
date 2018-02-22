#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <cmath>

class BotController{
    private:
        ros::Subscriber pos_sub;
        ros::Publisher expl_pub;
        std_msgs::String status;
        double pos_x, pos_y, ori_z, ang_z;
        double goal_x, goal_y;
        double dist_x, dist_y;

    public:
        BotController(ros::NodeHandle &nh){
            goal_x = 4.0;
            goal_y = 4.0;
            pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
            expl_pub = nh.advertise<std_msgs::String>("/explorer_status",1);
        }
        void callback( const nav_msgs::OdometryConstPtr& poseMsg){
            pos_x = poseMsg->pose.pose.position.x;
            pos_y = poseMsg->pose.pose.position.y;
            ori_z = poseMsg->pose.pose.orientation.z;
            ang_z = ori_z*2.19;

            dist_x = abs ( pos_x - goal_x );
            dist_y = abs ( pos_y - goal_y );
            if ( dist_x < 0.2 && dist_y < 0.2 )
              status.data = "goal";
            else
              status.data = "nope";

            expl_pub.publish(status);
            std::cout<< std::setprecision(2) << std::fixed;
            std::cout << poseMsg->header.stamp
                      << " Curr:" << pos_x << ", " << pos_y
                      << " Trgt:" << goal_x << ", " << goal_y
                      << " Dist:" << dist_x << ", " << dist_y << std::endl;
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "explorer");
    ros::NodeHandle nh;
    BotController bc(nh);

    ros::spin();
    return 0;
}
