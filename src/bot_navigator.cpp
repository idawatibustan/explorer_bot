#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class BotController{
    private:
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;
        ros::ServiceServer move_x;
        ros::ServiceServer move_y;
        double trans_x, trans_z;
        double pos_x, pos_y, ori_z, ang_z;
        double target_x, target_y, target_o, target_r;
        double init_r, init_ang_z, init_x, init_y;

    public:
        BotController(ros::NodeHandle &nh){
            trans_x = 0;
            trans_z = 0;
            pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
            vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
            ros::ServiceServer move_x = nh.advertiseService("move_x", move_x_callback);
            ros::ServiceServer move_y = nh.advertiseService("move_y", move_y_callback);
        }

        bool move_x_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
        {
            target_x += 1.0;
            return true;
        }

        bool move_y_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
        {
            target_y += 1.0;
            return true;
        }

        void callback( const nav_msgs::OdometryConstPtr& poseMsg){
            double PI_ = 3.1415;
            geometry_msgs::Twist base_cmd;
            pos_x = poseMsg->pose.pose.position.x;
            pos_y = poseMsg->pose.pose.position.y;
            ori_z = poseMsg->pose.pose.orientation.z;
            ang_z = ori_z*2.19;

            ang_n = ori_z*2.19; // hardcoded north direction
            ang_e = PI_/2; // hardcoded east direction
            // TODO : south direction
            // TODO : west direction

            target_x = pos_x;
            target_o = ang_z;

            if ( abs(target_x - pos_x) > 0.1 ) {
                // TODO: ensure the robot is facing North
                trans_x = 0.1;
            } else {
                // flag that goal is achieved
                trans_x = 0;
            }
            if ( abs(target_y - pos_y) > 0.1 ) {
                // TODO: ensure the robot is facing East
                trans_x = 0.1;
            } else {
                // flag that goal is achieved
                trans_x = 0;
            }

            // if( abs(ang_z - target_o) > 0.1 ) {
            //     trans_z = 0.1; // Change robot angular velocity
            // } else {
            //     trans_z = 0;
            // }

            // send speed instruction, turning before translating
            base_cmd.linear.x = trans_x;
            if(turn == 1){
              base_cmd.angular.z = trans_z;
            }

            vel_pub.publish(base_cmd);
            std::cout<< std::setprecision(2) << std::fixed;
            std::cout << poseMsg->header.stamp
                      << " Current:" << pos_x << "," << ori_z
                      << " Target:" << target_x << "," << ang_z
                      << " Move:" <<  trans_x << ", " << trans_z << std::endl;

        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "bot_navigator");
    ros::NodeHandle nh;
    BotController bc(nh);

    ros::spin();
    return 0;
}
