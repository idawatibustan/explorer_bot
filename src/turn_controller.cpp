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

  bool is_init, is_moving;

  int turn, multiplier;
  int move_n, move_s, move_e, move_w;
  int turn_n, turn_s, turn_e, turn_w;

  double kp_x, kp_z;
  double max_x, max_z;

  double trans_x, trans_z;
  double pos_x, pos_y;
  double target_x, target_y, target_z;
  double roll, pitch, yaw;
  double dx, dy, dt;

public:
  BotController(ros::NodeHandle &nh){
    is_init = true;
    is_moving = false;

    turn = 0;

    kp_x = 1;
    kp_z = -0.9;
    max_x = 0.5;
    max_z = 0.5;

    trans_x = 0;
    trans_z = 0;
    pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    mov_pub = nh.advertise<std_msgs::Bool>("/is_moving",1);

    move_goal = nh.advertiseService("move_goal", &BotController::move_goal_callback, this);

    turn_north = nh.advertiseService("turn_north", &BotController::turn_north_callback, this);
    turn_south = nh.advertiseService("turn_south", &BotController::turn_south_callback, this);
    turn_east = nh.advertiseService("turn_east", &BotController::turn_east_callback, this);
    turn_west = nh.advertiseService("turn_west", &BotController::turn_west_callback, this);
  }
  bool move_goal_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_goal");
    target_x = req.goal.x;
    target_y = req.goal.y;
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
    tf::Quaternion q(
      poseMsg->pose.pose.orientation.x,
      poseMsg->pose.pose.orientation.y,
      poseMsg->pose.pose.orientation.z,
      poseMsg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    yaw = angles::normalize_angle_positive(yaw);

    if(is_init) {
      target_x = pos_x;
      target_y = pos_y;
      target_z = yaw;
      is_init = false;
    }

    dx = target_x - pos_x;
    dy = target_y - pos_y;

    // if linear distance > threshold, change angle target
    if(std::abs(dx) > 0.1 && std::abs(dy) > 0.1){
      target_z = std::atan2( dy, dx );
      // target_z = angles::normalize_angle_positive(target_z);
    }

    dt = angles::shortest_angular_distance(target_z, yaw);
    std::cout << " dx= " << std::setw(5) << dx
              << " dy= " << std::setw(5) << dy
              << " d_t= " << std::setw(5) << dt
              << " atan= " << std::setw(5) << target_z << std::endl;

    // if angular distance > threshold, turn
    if(std::abs(dt) > 0.01){
      trans_z = kp_z * dt;
    } else {
      trans_z = 0.00;
    }

    // if angular distance is huge, do not move forward
    if (std::abs(dt) < 0.08) {
      trans_x = kp_x * dx;
    } else {
      trans_x = 0.00;
    }

    // set maximum magnitude of trans_x
    if( trans_x < 0 ) {
      trans_x = std::max(-max_x, trans_x);
    } else {
      trans_x = std::min(trans_x, max_x);
    }
    // set maximum magnitude of trans_z
    if( trans_z < 0 ) {
      trans_z = std::max(-max_z, trans_z);
    } else {
      trans_z = std::min(trans_z, max_z);
    }
    base_cmd.linear.x = trans_x;
    base_cmd.angular.z = trans_z;
    vel_pub.publish(base_cmd);
    // std::cout << " y= " << std::setw(5) << yaw
    //           << " t= " << std::setw(5) << target_z
    //           << " d_t= " << std::setw(5) << dt
    //           << " trans_z:" << trans_z << std::endl;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "bot_navigator");
  ros::NodeHandle nh;
  BotController bn(nh);

  ros::spin();
  return 0;
}
