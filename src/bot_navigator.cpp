#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>


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

  ros::ServiceServer move_north;
  ros::ServiceServer move_south;
  ros::ServiceServer move_east;
  ros::ServiceServer move_west;

  ros::ServiceServer turn_north;
  ros::ServiceServer turn_south;
  ros::ServiceServer turn_east;
  ros::ServiceServer turn_west;

  int count, turn;
  int move_n, move_s, move_e, move_w;
  int turn_n, turn_s, turn_e, turn_w;

  bool is_moving;

  double trans_x, trans_z;
  double pos_x, pos_y, ori_z, ang_z;
  double ang_n, ang_e, ang_s, ang_w;
  double target_x, target_y, target_o, target_r;
  double init_r, init_ang_z, init_x, init_y;

public:
  BotNavigator(ros::NodeHandle &nh){
    count = 0;
    turn = 0;

    move_n = 0;
    turn_n = 0;

    move_s = 0;
    turn_s = 0;

    move_e = 0;
    turn_e = 0;

    move_w = 0;
    turn_w = 0;

    is_moving = false;

    trans_x = 0;
    trans_z = 0;
    pos_sub = nh.subscribe("/odom",1,&BotNavigator::callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    mov_pub = nh.advertise<std_msgs::Bool>("/is_moving",1);

    move_north = nh.advertiseService("move_north", &BotNavigator::move_north_callback, this);
    move_south = nh.advertiseService("move_south", &BotNavigator::move_south_callback, this);
    move_east = nh.advertiseService("move_east", &BotNavigator::move_east_callback, this);
    move_west = nh.advertiseService("move_west", &BotNavigator::move_west_callback, this);

    turn_north = nh.advertiseService("turn_north", &BotNavigator::turn_north_callback, this);
    turn_south = nh.advertiseService("turn_south", &BotNavigator::turn_south_callback, this);
    turn_east = nh.advertiseService("turn_east", &BotNavigator::turn_east_callback, this);
    turn_west = nh.advertiseService("turn_west", &BotNavigator::turn_west_callback, this);
  }

  bool move_north_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_north");
    target_x = req.goal.x + 1.0;
    turn = 1;
    turn_n = 1;
    move_n = 1;
    is_moving = true;
    printf("I want to move north\n");
  }

  bool turn_north_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_north");
    turn = 1;
    turn_n = 1;
    is_moving = true;
    printf("I want to turn to north\n");
  }

  bool move_south_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested movve_south");
    target_x = req.goal.x - 1.0;
    turn = 1;
    turn_s = 1;
    move_s = 1;
    is_moving = true;
    printf("I want to turn and move south\n");
    return true;
  }

  bool turn_south_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_south");
    turn = 1;
    turn_s = 1;
    is_moving = true;
    printf("I want to turn to south\n");
  }


  bool move_east_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_east");
    target_y = req.goal.y - 1.0;
    turn = 1;
    turn_e = 1;
    move_e = 1;
    is_moving = true;
    printf("I want to turn and move east\n");
    return true;
  }

  bool turn_east_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_east");
    turn = 1;
    turn_e = 1;
    is_moving = true;
    printf("I want to turn to east\n");
  }

  bool move_west_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested movve_west");
    target_y = req.goal.y + 1.0;
    turn = 1;
    turn_w = 1;
    move_w = 1;
    is_moving = true;
    printf("I want to turn and move west\n");
    return true;
  }

  bool turn_west_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_west");
    turn = 1;
    turn_w = 1;
    is_moving = true;
    printf("I want to turn to west\n");
  }


  void callback( const nav_msgs::OdometryConstPtr& poseMsg){
    double PI_ = 3.1415;
    geometry_msgs::Twist base_cmd;
    std_msgs::Bool moving;
    pos_x = poseMsg->pose.pose.position.x;
    pos_y = poseMsg->pose.pose.position.y;
    ori_z = poseMsg->pose.pose.orientation.z;
    ang_z = ori_z*2.19;

    ang_n = 0.002479;  // the value is 0
    ang_e = -1.580881;      // the value is -1.580981
    ang_w = 1.580981;       // the value is 1.580981
    ang_s = -2.189983;           // the value is 2.189980 or -2.189980

    if(count == 0){ // initialisation process
      init_x = poseMsg->pose.pose.position.x;
      init_y = poseMsg->pose.pose.position.y;
      init_ang_z = ori_z*2.19;
      count = 1;
      turn = 0;

      move_n = 0;
      move_s = 0;
      move_e = 0;
      move_w = 0;

      turn_n = 0;
      turn_s = 0;
      turn_e = 0;
      turn_w = 0;

      target_x = pos_x;
      target_y = pos_y;
      target_o = ang_z;
      printf("Initialiation Done init_x = %f \n", init_x);
      printf("Orientation of Turtlebot = %f \n", init_ang_z);
    }// initialise the positition X and angular z

    if(move_n == 1){ //flag to move north //moving in the north direction
      if(turn_n == 1){
        if(ang_z < ang_n){ //turning to north
          trans_z = 0.5;
        }
        else{
          trans_z = 0;
          turn_n = 0;
          printf("I am done facing north, my ang_z = %f\n", ang_z);
          printf("I do not want to turn turn_n = %d\n", turn_n);
        }
      }
      else
      {
        if(pos_x < target_x){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("I am Moving Forward, pose_x = %f \n", pos_x);
        }
        else{
          trans_x = 0;
          move_n = 0;
          is_moving = false;
          printf("move_n = %d\nturn_n = %d\n", move_n, turn_n);
        }
      }
    }


    if(turn_n == 1){ //flag to turn north
      if( fabs(ang_z) > ang_n){ //turning to north
        trans_z = 0.5;
      }
      else{
        trans_z = 0;
        turn_n = 0;
        is_moving = false;
        printf("my ang_n = %f\n", ang_n);
        printf("I am done facing north, my ang_z = %f\n", ang_z);
        printf("I do not want to turn turn_n = %d\n", turn_n);
      }
    }

    if(move_s == 1){ //flag to move move_east
      if(turn_s == 1){
        if(ang_z > ang_s){
          trans_z = -0.5;
          printf("I am Turning South\n");
          printf("My ang_z = %f\n", ang_z);
        }
        else{
          trans_z = 0;
          turn_s = 0;
          printf("I am done facing south, my ang_z = %f\n", ang_z);
          printf("I do not want to turn turn_s = %d\n", turn_s);
        }
      }
      else{
        if(pos_x > target_x){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("I am Moving Forward, pose_y = %f \n", pos_y);
          printf("init_y = %f \n", init_y);
        }
        else{
          trans_x = 0;
          move_s = 0;
          is_moving = false;
          printf("move_s = %d\nturn_s = %d\n", move_s, turn_s);
        }
      }
    }


    if(turn_s == 1){ //flag to turn to south
      if(ang_z > ang_s){
        trans_z = -0.5;
        printf("I am Turning South\n");
        printf("My ang_z = %f\n", ang_z);
      }
      else{
        trans_z = 0;
        turn_s = 0;
        is_moving = false;
        printf("I am done facing south, my ang_z = %f\n", ang_z);
        printf("I do not want to turn turn_s = %d\n", turn_s);
      }
    }


    if(move_e == 1){ //flag to move move_east
      if(turn_e == 1){
        if(ang_z > ang_e){
          trans_z = -0.5;
          printf("I am Turning\n");
          printf("My ang_z = %f\n", ang_z);
        }
        else{
          trans_z = 0;
          turn_e = 0;
          printf("I am done facing east my ang_z = %f\n", ang_z);
          printf("I do not want to turn turn_e = %d\n", turn_e);
        }
      }
      else{
        if(pos_y > target_y){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("I am Moving Forward, pose_y = %f \n", pos_y);
          printf("init_y = %f \n", init_y);
        }
        else{
          trans_x = 0;
          move_e = 0;
          is_moving = false;
          printf("move_e = %d\nturn_e = %d\n", move_e, turn_e);
        }
      }
    }


    if(turn_e == 1){
      if(ang_z > ang_e){
        trans_z = -0.5;
        printf("I am Turning\n");
        printf("My ang_z = %f\n", ang_z);
      }
      else{
        trans_z = 0;
        turn_e = 0;
        is_moving = false;
        printf("my ang_e = %f\n", ang_e);
        printf("I am done facing east my ang_z = %f\n", ang_z);
        printf("I do not want to turn turn_e = %d\n", turn_e);
      }
    }

    if(move_w == 1){ //flag to move move_west
      if(turn_w == 1){
        if(ang_z < ang_w){
          trans_z = 0.5;
          printf("I am Turning West\n");
          printf("My ang_z = %f\n", ang_z);
        }
        else{
          trans_z = 0;
          turn_w = 0;
          printf("I am done facing west, my ang_z = %f\n", ang_z);
          printf("I do not want to turn turn_w = %d\n", turn_w);
        }
      }
      else{
        if(pos_y < target_y){
          trans_x = 0.1;//*dist; // Change robot velocity
          printf("I am Moving Forward, pose_y = %f \n", pos_y);
          printf("init_y = %f \n", init_y);
        }
        else{
          trans_x = 0;
          move_w = 0;
          is_moving = false;
          printf("move_s = %d\nturn_s = %d\n", move_w, turn_w);
        }
      }
    }

    if(turn_w == 1){
      if(ang_z < ang_w){
        trans_z = 0.5;
        printf("I am Turning West\n");
        printf("My ang_z = %f\n", ang_z);
      }
      else{
        trans_z = 0;
        turn_w = 0;
        is_moving = false;
        printf("my ang_w = %f\n", ang_w);
        printf("I am done facing west, my ang_z = %f\n", ang_z);
        printf("I do not want to turn turn_w = %d\n", turn_w);
      }
    }

    base_cmd.linear.x = trans_x;
    if(turn == 1){
      base_cmd.angular.z = trans_z;
    }
    vel_pub.publish(base_cmd);

    moving.data = is_moving;
    mov_pub.publish(moving);
    std::cout<< std::setprecision(2) << std::fixed;
    //std::cout << poseMsg->header.stamp
    //  << " C:" << pos_x << "," << pos_y << "," << ori_z
    //  << " T:" << target_x << "," << target_y << "," << target_o
    //  << " M:" <<  trans_x << ", " << trans_z << std::endl;

  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "bot_navigator");
  ros::NodeHandle nh;
  BotNavigator bn(nh);

  ros::spin();
  return 0;
}
