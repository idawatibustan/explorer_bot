#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <angles/angles.h>

#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include <explorer_bot/MoveGoal.h>

class BotController{
private:
  ros::Subscriber pos_sub;
  ros::Publisher vel_pub;
  ros::Publisher mov_pub;

  ros::ServiceServer move_goal;

  ros::ServiceServer turn_north;
  ros::ServiceServer turn_south;
  ros::ServiceServer turn_east;
  ros::ServiceServer turn_west;

  bool is_moving;

  int count, turn, multiplier;
  int move_n, move_s, move_e, move_w;
  int turn_n, turn_s, turn_e, turn_w;

  double trans_x, trans_z;
  double pos_x, pos_y, ori_z;
  double target_x, target_y, target_z;
  double roll, pitch, yaw;
  double dt;
  double kp_z;
  double min_z;

public:
  BotController(ros::NodeHandle &nh){
    count = 0;
    turn = 0;

    kp_z = -0.8;
    min_z = 0.4;
    is_moving = false;

    trans_x = 0;
    trans_z = 0;
    pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    mov_pub = nh.advertise<std_msgs::Bool>("/is_moving",1);

    turn_north = nh.advertiseService("turn_north", &BotController::turn_north_callback, this);
    turn_south = nh.advertiseService("turn_south", &BotController::turn_south_callback, this);
    turn_east = nh.advertiseService("turn_east", &BotController::turn_east_callback, this);
    turn_west = nh.advertiseService("turn_west", &BotController::turn_west_callback, this);
  }
  bool turn_north_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_north");
    target_z = 0.00;
  }
  bool turn_south_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_south");
    target_z = M_PI;
  }
  bool turn_east_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_east");
    target_z = 1.5 * M_PI;
  }
  bool turn_west_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_west");
    target_z = 0.5 * M_PI;
  }

  void callback( const nav_msgs::OdometryConstPtr& poseMsg){
    geometry_msgs::Twist base_cmd;
    pos_x = poseMsg->pose.pose.position.x;
    pos_y = poseMsg->pose.pose.position.y;
    ori_z = poseMsg->pose.pose.orientation.z;

    tf::Quaternion q(
      poseMsg->pose.pose.orientation.x,
      poseMsg->pose.pose.orientation.y,
      poseMsg->pose.pose.orientation.z,
      poseMsg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    yaw = angles::normalize_angle_positive(yaw);
    if(count == 0) {
      target_z = yaw;
      count = 1;
    }

    dt = angles::shortest_angular_distance(target_z, yaw);
    if(std::abs(dt) > 0.001){
      trans_z = kp_z * dt;
    } else {
      trans_z = 0.00;
    }

    // base_cmd.linear.x = trans_x;
    if( trans_z < 0 ) {
      trans_z = std::max(-min_z, trans_z);
    }
    else {
      trans_z = std::min(trans_z, min_z);
    }
    base_cmd.angular.z = trans_z;
    vel_pub.publish(base_cmd);
    std::cout << " y= " << std::setw(5) << yaw
              << " t= " << std::setw(5) << target_z
              << " d_x= " << std::setw(5) << dt
              << " trans_z:" << trans_z << std::endl;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "bot_navigator");
  ros::NodeHandle nh;
  BotController bn(nh);

  ros::spin();
  return 0;
}
