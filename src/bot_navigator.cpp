#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <angles/angles.h>

#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>

#include <explorer_bot/MoveGoal.h>

class BotNavigator{
private:
  const double dt_threshold_high = 0.05;
  const double dt_threshold_low = 0.01;
  const double max_x = 0.5;
  const double max_z = 0.5;
  const double kp_x = 0.8;
  const double kp_z = -1.2;

  ros::Subscriber pos_sub;
  ros::Publisher vel_pub;
  ros::Publisher mov_pub;

  ros::ServiceServer turn_north, turn_east, turn_south, turn_west;

  ros::ServiceServer move_north;
  ros::ServiceServer move_south;
  ros::ServiceServer move_east;
  ros::ServiceServer move_west;

  bool is_moving;
  int count, turn;
  int move_n, move_s, move_e, move_w;
  double dx_threshold, dt_threshold, dv_threshold;

  double trans_x, trans_z;
  double prev_trans_x, prev_trans_z;
  double pos_x, pos_y;
  double target_x, target_y, target_z;
  double roll, pitch, yaw;
  double dx, dy, dt;

public:
  BotNavigator(ros::NodeHandle &nh){
    count = 0;
    turn = 0;
    is_moving = false;

    dt_threshold = dt_threshold_low;
    dx_threshold = 0.01;
    dv_threshold = 0.1;

    trans_x = 0;
    trans_z = 0;
    prev_trans_x = trans_x;
    prev_trans_z = trans_z;

    pos_sub = nh.subscribe("/odom",1,&BotNavigator::callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    mov_pub = nh.advertise<std_msgs::Bool>("/is_moving",1);

    turn_north = nh.advertiseService("turn_north", &BotNavigator::turn_north_callback, this);
    turn_south = nh.advertiseService("turn_south", &BotNavigator::turn_south_callback, this);
    turn_east = nh.advertiseService("turn_east", &BotNavigator::turn_east_callback, this);
    turn_west = nh.advertiseService("turn_west", &BotNavigator::turn_west_callback, this);

    move_north = nh.advertiseService("move_north", &BotNavigator::move_north_callback, this);
    move_south = nh.advertiseService("move_south", &BotNavigator::move_south_callback, this);
    move_east = nh.advertiseService("move_east", &BotNavigator::move_east_callback, this);
    move_west = nh.advertiseService("move_west", &BotNavigator::move_west_callback, this);
  }
  bool turn_north_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_north");
    target_z = 0.00;
    is_moving = true;
  }
  bool turn_south_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_south");
    target_z = M_PI;
    is_moving = true;
  }
  bool turn_east_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_east");
    target_z = 1.5 * M_PI;
    is_moving = true;
  }
  bool turn_west_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_west");
    target_z = 0.5 * M_PI;
    is_moving = true;
  }
  bool move_north_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_north");
    target_z = 0.00;
    target_x = req.goal.x + 1.0;
    turn = 1;
    move_n = 1;
  }
  bool move_south_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_south");
    target_z = M_PI;
    target_x = req.goal.x - 1.0;
    turn = 1;
    move_s = 1;
  }
  bool move_east_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_east");
    target_z = 1.5 * M_PI;
    target_y = req.goal.y - 1.0;
    turn = 1;
    move_e = 1;
  }
  bool move_west_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_west");
    target_z = 0.5 * M_PI;
    target_y = req.goal.y + 1.0;
    turn = 1;
    move_w = 1;
  }
  void threshold_up() { this->dt_threshold = this->dt_threshold_high; };
  void threshold_down() { this->dt_threshold = this->dt_threshold_low; };
  void callback( const nav_msgs::OdometryConstPtr& poseMsg){
    geometry_msgs::Twist base_cmd;
    std_msgs::Bool moving;
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

    if(count == 0){ // initialisation process
      count = 1;
      turn = 0;
      // initialize target to pos keep the robot at starting point
      target_x = pos_x;
      target_y = pos_y;
      target_z = yaw;
      printf("Initialiation pose = %f, %f\n", pos_x, pos_y);
      printf("Orientation of Turtlebot = %f \n", yaw);
    }
    // get difference from position to from target
    dx = std::abs(target_x - pos_x);
    dy = std::abs(target_y - pos_y);
    dt = angles::shortest_angular_distance(target_z, yaw);

    // keep the robot to face the intended direction when moving forward
    if(std::abs(dt) > this->dt_threshold){
      trans_z = kp_z * dt;
      turn = 1;
      is_moving = true;
    } else {
      trans_z = 0.00;
      turn = 0;
      is_moving = false;
    }

    // moving towords north
    if(move_n == 1){
      is_moving = true;
      // do not move until turning to intended direction
      if(turn == 0){
        threshold_up(); // toggle up threshold to reduce bumpiness
        if(pos_x < target_x && dx > this->dx_threshold) {
          trans_x = kp_x * dx; // proportionate control to target speed
          printf("Moving, pos_x= %f, target_x= %f \n", pos_x, target_x);
        }
        else{
          // target distance reached, stop movement, set flags to false
          trans_x = 0;
          move_n = 0;
          is_moving = false;
          threshold_down();
          printf("move_n = %d\nturn = %d\n", move_n, turn);
        }
      } else {
        trans_x = 0;
      }
    }
    // moving towards south
    if(move_s == 1){
      is_moving = true;
      if(turn == 0) {
        threshold_up();
        if(pos_x > target_x && dx > this->dx_threshold){
          trans_x = kp_x * dx;
          printf("Moving, pos_x= %f, target_x= %f \n", pos_x, target_x);
        }
        else{
          trans_x = 0;
          move_s = 0;
          is_moving = false;
          threshold_down();
          printf("move_s = %d\nturn = %d\n", move_s, turn);
        }
      } else {
        trans_x = 0;
      }
    }
    // moving towards east
    if(move_e == 1){
      is_moving = true;
      if(turn == 0) {
        threshold_up();
        if(pos_y > target_y && dy > this->dx_threshold){
          trans_x = kp_x * dy;
          printf("Moving, pos_y= %f, target_y= %f \n", pos_y, target_y);
        }
        else{
          trans_x = 0;
          move_e = 0;
          is_moving = false;
          threshold_down();
          printf("move_e = %d\n turn = %d\n", move_e, turn);
        }
      } else {
        trans_x = 0;
      }
    }
    // moving towards west
    if(move_w == 1){
      is_moving = true;
      if(turn == 0) {
        threshold_up();
        if(pos_y < target_y && dy > this->dx_threshold){
          trans_x = kp_x * dy; //*dist; // Change robot velocity
          printf("Moving, pos_y= %f, target_y= %f \n", pos_y, target_y);
        }
        else{
          trans_x = 0;
          move_w = 0;
          is_moving = false;
          threshold_down();
          printf("move_s = %d\nturn = %d\n", move_w, turn);
        }
      } else {
        trans_x = 0;
      }
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

    // get increase in speed of trans_x & trans_z
    double dv_x = trans_x - prev_trans_x;
    double dv_z = std::abs(trans_z - prev_trans_z);
    // simple controller max acceleration of dv_threshold
    if ( dv_x > 0 ) { // for trans_x
      if ( dv_x > this->dv_threshold ) {
        prev_trans_x += this->dv_threshold;
      } else {
        prev_trans_x = trans_x;
      }
    } else {
      if ( dv_x < -this->dv_threshold ) {
        prev_trans_x -= this->dv_threshold;
      } else {
        prev_trans_x = trans_x;
      }
    }
    // acceleration control for trans_z
    if ( dv_z > this->dv_threshold ) {
      if (trans_z < 0) {
        prev_trans_z -= this->dv_threshold;
      } else {
        prev_trans_z += this->dv_threshold;
      }
    } else {
      prev_trans_z = trans_z;
    }

    // send trans_x & trans_z
    base_cmd.linear.x = prev_trans_x;
    base_cmd.angular.z = prev_trans_z;
    vel_pub.publish(base_cmd);
    // publish if robot is_moving
    moving.data = is_moving;
    mov_pub.publish(moving);

    // print if robot is moving (monitoring purposes)
    if (trans_x + std::abs(trans_z) > 0) {
      std::cout
      // << "P:" << pos_x << "," << pos_y << "," << yaw
      // << " T:" << target_x << "," << target_y << "," << target_z
      // << " d:" << dx << "," << dy
      // << " dt:" << std::abs(dt)
      // << " v:" <<  trans_x << ", " << trans_z
      // << " dv:" <<  dv_x << ", " << dv_z
      << " tx:" << std::setw(5) << prev_trans_x
      << " tz:" << std::setw(5) << prev_trans_z
      << std::endl;
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "bot_navigator");
  ros::NodeHandle nh;
  BotNavigator bn(nh);

  ros::spin();
  return 0;
}
