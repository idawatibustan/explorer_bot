#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>


#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class BotNavigator{
    private:
        ros::Subscriber pos_sub;
        ros::Publisher vel_pub;
        ros::ServiceServer move_x;
        ros::ServiceServer move_y;
        bool init_flag;
        double trans_x, trans_z;
        double pos_x, pos_y, ori_z, ang_z;
        double ang_n, ang_e, ang_s, ang_w;
        double target_x, target_y, target_o, target_r;
        double init_r, init_ang_z, init_x, init_y;

    public:
        BotNavigator(ros::NodeHandle &nh){
            init_flag = true;
            trans_x = 0;
            trans_z = 0;
            pos_sub = nh.subscribe("/odom",1,&BotNavigator::callback, this);
            vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
            move_x = nh.advertiseService("move_x", &BotNavigator::move_x_callback, this);
            move_y = nh.advertiseService("move_y", &BotNavigator::move_y_callback, this);
        }

        bool move_x_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
        {
            ROS_INFO("requested move_x");
            target_x += 1.0;
            return true;
        }

        bool move_y_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
        {
            ROS_INFO("requested move_y");
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

            if (init_flag) {
                target_x = pos_x;
                target_y = pos_y;
                target_o = ang_z;
                init_flag = false;
            }

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
            // if(turn == 1){
            //   base_cmd.angular.z = trans_z;
            // }

            vel_pub.publish(base_cmd);
            std::cout<< std::setprecision(2) << std::fixed;
            std::cout << poseMsg->header.stamp
                      << " C:" << pos_x << "," << pos_y << "," << ori_z
                      << " T:" << target_x << "," << target_y << "," << target_o
                      << " M:" <<  trans_x << ", " << trans_z << std::endl;

        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "bot_navigator");
    ros::NodeHandle nh;
    BotNavigator bn(nh);

    ros::spin();
    return 0;
}
