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
  ros::Subscriber pos_sub;
  ros::Publisher vel_pub;
  ros::Publisher mov_pub;

  ros::ServiceServer turn_north, turn_east, turn_south, turn_west;

  ros::ServiceServer move_north;
  ros::ServiceServer move_south;
  ros::ServiceServer move_east;
  ros::ServiceServer move_west;

  int count, turn;
  int move_n, move_s, move_e, move_w;
  int turn_n, turn_s, turn_e, turn_w;

  bool is_moving;

  double kp_x, kp_z;
  double max_x, max_z;

  double trans_x, trans_z;
  double pos_x, pos_y, ori_z, ang_z;
  double ang_n, ang_e, ang_s, ang_w;
  double target_x, target_y, target_z;
  double init_r, init_ang_z, init_x, init_y;
  double roll, pitch, yaw;
  double dx, dy, dt;

public:
  BotNavigator(ros::NodeHandle &nh){
    count = 0;
    turn = 0;

    is_moving = false;

    kp_z = -0.9;
    max_z = 0.5;

    trans_x = 0;
    trans_z = 0;

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
    is_moving = true;
    printf("I want to move north\n");
  }
  bool move_south_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_south");
    target_z = M_PI;
    target_x = req.goal.x - 1.0;
    turn = 1;
    move_s = 1;
    is_moving = true;
    printf("I want to turn and move south\n");
    return true;
  }

  bool move_east_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_east");
    target_z = 1.5 * M_PI;
    target_y = req.goal.y - 1.0;
    turn = 1;
    move_e = 1;
    is_moving = true;
    printf("I want to turn and move east\n");
    return true;
  }

  bool move_west_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_west");
    target_z = 0.5 * M_PI;
    target_y = req.goal.y + 1.0;
    turn = 1;
    move_w = 1;
    is_moving = true;
    printf("I want to turn and move west\n");
    return true;
  }

  void callback( const nav_msgs::OdometryConstPtr& poseMsg){
    double PI_ = 3.1415;
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


      target_x = pos_x;
      target_y = pos_y;
      target_z = yaw;
      printf("Initialiation Done init_x = %f \n", init_x);
      printf("Orientation of Turtlebot = %f \n", init_ang_z);
    } // initialise the positition X and angular z

    dt = angles::shortest_angular_distance(target_z, yaw);

    int turn;
    if(std::abs(dt) > 0.01){
      trans_z = kp_z * dt;
      turn = 1;
      is_moving = true;
    } else {
      trans_z = 0.00;
      turn = 0;
      is_moving = false;
    }

    if(move_n == 1){ //flag to move north //moving in the north direction
      is_moving = true;
      if(turn == 0){
        if(pos_x < target_x) {
          trans_x = 0.1; //*dist; // Change robot velocity
          printf("Moving, pos_x= %f, target_x= %f \n", pos_x, target_x);
        }
        else{
          trans_x = 0;
          move_n = 0;
          is_moving = false;
          printf("move_n = %d\nturn = %d\n", move_n, turn);
        }
      } else {
        trans_x = 0;
      }
    }

    if(move_s == 1){ //flag to move move_east
      is_moving = true;
      if(turn == 0) {
        if(pos_x > target_x){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("Moving, pos_x= %f, target_x= %f \n", pos_x, target_x);
        }
        else{
          trans_x = 0;
          move_s = 0;
          is_moving = false;
          printf("move_s = %d\nturn = %d\n", move_s, turn);
        }
      } else {
        trans_x = 0;
      }
    }

    if(move_e == 1){ //flag to move move_east
      is_moving = true;
      if(turn == 0) {
        if(pos_y > target_y){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("Moving, pos_y= %f, target_y= %f \n", pos_y, target_y);
        }
        else{
          trans_x = 0;
          move_e = 0;
          is_moving = false;
          printf("move_e = %d\n turn = %d\n", move_e, turn);
        }
      } else {
        trans_x = 0;
      }
    }

    if(move_w == 1){ //flag to move move_west
      is_moving = true;
      if(turn == 0) {
        if(pos_y < target_y){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("Moving, pos_y= %f, target_y= %f \n", pos_y, target_y);
        }
        else{
          trans_x = 0;
          move_w = 0;
          is_moving = false;
          printf("move_s = %d\nturn = %d\n", move_w, turn);
        }
      } else {
        trans_x = 0;
      }
    }

    // set maximum magnitude of trans_z
    if( trans_z < 0 ) {
      trans_z = std::max(-max_z, trans_z);
    } else {
      trans_z = std::min(trans_z, max_z);
    }

    base_cmd.linear.x = trans_x;
    if(turn == 1){
      base_cmd.angular.z = trans_z;
    }
    vel_pub.publish(base_cmd);

    moving.data = is_moving;
    mov_pub.publish(moving);
    std::cout<< std::setprecision(2) << std::fixed;
    // std::cout
    //  << " C:" << pos_x << "," << pos_y << "," << ori_z
    // << " T:" << target_x << "," << target_y << "," << target_z
    //  << " M:" <<  trans_x << ", " << trans_z
    // << std::endl;

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "bot_navigator");
  ros::NodeHandle nh;
  BotNavigator bn(nh);

  ros::spin();
  return 0;
}
