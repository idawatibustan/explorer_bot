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

  ros::ServiceServer move_north;
  ros::ServiceServer move_south;
  ros::ServiceServer move_east;
  ros::ServiceServer move_west;

  ros::ServiceServer turn_north;
  ros::ServiceServer turn_south;
  ros::ServiceServer turn_east;
  ros::ServiceServer turn_west;

  bool is_moving;

  int count, turn;
  int move_x, move_y;

  double multiplier;

  double trans_x, trans_z;
  double pos_x, pos_y, ori_z, ang_z;
  double ang_n, ang_e, ang_s, ang_w;
  double target_x, target_y, target_z, target_o;
  double init_ang_z, init_x, init_y;
  double roll, pitch, yaw;
  double x_1, y_1, x_2, y_2;
  double change_in_x, change_in_y;
  double shortest_angular_distance;
  double dt, kp_z, min_z;

public:
  BotController(ros::NodeHandle &nh){
    count = 0;
    turn = 0;

    min_z = 0.4;
    is_moving = false;

    move_x = 0;
    move_y = 0;

    trans_x = 0;
    trans_z = 0;
    pos_sub = nh.subscribe("/odom",1,&BotController::callback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
    mov_pub = nh.advertise<std_msgs::Bool>("/is_moving", 1);

    move_goal = nh.advertiseService("move_goal", &BotController::move_goal_callback, this);

    move_north = nh.advertiseService("move_north", &BotController::move_north_callback, this);
    move_south = nh.advertiseService("move_south", &BotController::move_south_callback, this);
    move_east = nh.advertiseService("move_east", &BotController::move_east_callback, this);
    move_west = nh.advertiseService("move_west", &BotController::move_west_callback, this);

    turn_north = nh.advertiseService("turn_north", &BotController::turn_north_callback, this);
    turn_south = nh.advertiseService("turn_south", &BotController::turn_south_callback, this);
    turn_east = nh.advertiseService("turn_east", &BotController::turn_east_callback, this);
    turn_west = nh.advertiseService("turn_west", &BotController::turn_west_callback, this);
  }

  bool move_goal_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_goal");
  }

  bool move_north_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_north");
    target_x = req.goal.x + 1.0;
    turn = 1;
    printf("I want to move north\n");
    x_2 = 1.0;
    y_2 = 0.0;
    target_z = 0.00;
    move_x = 1;
    move_y = 0;
    is_moving = true;
    return true;
  }

  bool turn_north_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_north");
    turn = 1;
    target_z = 0.00;
    printf("I want to turn to north\n");
    x_2 = 1;
    y_2 = 0;
    is_moving = true;
    return true;
  }

  bool move_south_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested movve_south");
    target_x = req.goal.x - 1.0;
    turn = 1;
    printf("I want to turn and move south\n");
    target_z = 180.00;
    x_2 = -1,0;
    y_2 = 0.0;
    move_x = 1;
    move_y = 0;
    is_moving = true;
    return true;
  }


  bool turn_south_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_south");
    turn = 1;
    printf("I want to turn to south\n");
    x_2 = -1.0;
    y_2 = 0.0;
    target_z = 180.00;
    is_moving = true;
    return true;
  }


  bool move_east_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested move_east");
    target_y = req.goal.y - 1.0;
    turn = 1;
    printf("I want to turn and move east\n");
    target_z = 270.00;
    x_2 =  0.0;
    y_2 = -1.0;
    move_x = 0;
    move_y = 1;
    is_moving = true;
    return true;
  }

  bool turn_east_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_east");
    turn = 1;
    x_2 = 0;
    y_2 = -1;
    printf("I want to turn to east\n");
    is_moving = true;
    target_z = 270.00;
  }

  bool move_west_callback( explorer_bot::MoveGoal::Request& req, explorer_bot::MoveGoal::Response& res )
  {
    ROS_INFO("requested movve_west");
    target_y = req.goal.y + 1.0;
    turn = 1;
    printf("I want to turn and move west\n");
    target_z = 90.00;
    x_2 = 0.0;
    y_2 = 1.0;
    move_x = 0;
    move_y = 1;
    is_moving = true;
    return true;
  }

  bool turn_west_callback( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
  {
    ROS_INFO("requested turn_west");
    turn = 1;
    x_2 = 0;
    y_2 = 1;
    printf("I want to turn to west\n");
    is_moving = true;
    target_z = 90.00;
  }

  void callback( const nav_msgs::OdometryConstPtr& poseMsg){
    double PI_ = 3.1415;
    geometry_msgs::Twist base_cmd;
    pos_x = poseMsg->pose.pose.position.x;
    pos_y = poseMsg->pose.pose.position.y;
    ori_z = poseMsg->pose.pose.orientation.z;
    //ang_z = ori_z*2.19;
    //x_1 = x_2;
    //y_1 = y_2;
    //std::cout << "x_1 =" << x_1 << '\n';
    //std::cout << "y_1 =" << y_1 << '\n';
    if(count == 0){ // initialisation process
      init_x = poseMsg->pose.pose.position.x;
      init_y = poseMsg->pose.pose.position.y;

      turn = 1; //change to 0 after testing
      multiplier = 0;
      move_x = 0;

      x_1 = init_x;
      y_1 = init_y;

      x_2 = init_x;
      y_2 = init_y;

      target_z = 0.00;

      target_x = pos_x;
      target_y = pos_y;
      target_o = ang_z;
      count = 1;

    }// initialise the positition X and angular z

    tf::Quaternion q(poseMsg->pose.pose.orientation.x,
      poseMsg->pose.pose.orientation.y,
      poseMsg->pose.pose.orientation.z,
      poseMsg->pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      m.getRPY(roll,pitch,yaw);
      yaw = angles::normalize_angle_positive(yaw);
      ang_z = yaw * 180/M_PI;
      //std::cout << "ang_z = " << ang_z << '\n';
      change_in_x = x_2 - x_1;
      change_in_y = y_2 - y_1;
      //std::cout << "change_in_x = " << change_in_x << '\n';
      //std::cout << "change_in_y = " << change_in_y << '\n';

      if (change_in_x == 0 && change_in_y == 0) //Origin
      {
        std::cout << "(" << change_in_x << "," << change_in_y << ") is the origin." << "\n";
      }

      else
      if(change_in_x == 1.0 && change_in_y == 0.0) //Traget = North
      {
        std::cout << "(" << change_in_x << "," << change_in_y << ") Target is North." << "\n";
      }

      else
      if(change_in_x == -1 && change_in_y == 0.0) // Target = South
      {
        std::cout << "(" << change_in_x << "," << change_in_y << ") Target is South." << "\n";
      }

      else
      if(change_in_y == 1 && change_in_x == 0.0) // Target = West
      {
        std::cout << "(" << change_in_x << "," << change_in_y << ") Target is West." << "\n";
      }
      else

      if(change_in_y == -1 && change_in_x == 0) // Target = East
      {
        std::cout << "(" << change_in_x << "," << change_in_y << ") Target is East." << "\n";
      }

      else
      if(change_in_x > 0 && change_in_y < 0) /* check for quadrant I */
      {
        std::cout <<"("<<change_in_x <<","<< change_in_y <<")is in quadrant I" <<"\n";
        target_z = 360 + (atan((y_2 - y_1)/(x_2 - x_1)) * 180/PI_);
      }

      else
      if(change_in_x > 0 && change_in_y > 0) /* check for quadrant II */
      {
        std::cout<<"("<<change_in_x <<","<< change_in_y <<")is in quadrant II" <<"\n";
        target_z = (atan((y_2 - y_1)/(x_2 - x_1)) * 180/PI_);
      }

      else
      if(change_in_x < 0 && change_in_y > 0) /* check for quadrant III */
      {
        std::cout<<"("<<change_in_x <<","<< change_in_y <<")is in quadrant III" <<"\n";
        target_z = 180 + (atan((y_2 - y_1)/(x_2 - x_1)) * 180/PI_);
      }

      else
      if(change_in_x < 0 && change_in_y < 0) /* check for quadrant IV */
      {
        std::cout<<"("<<change_in_x <<","<<change_in_y <<")is in quadrant IV" <<"\n";
        target_z = 180 + (atan((y_2 - y_1)/(x_2 - x_1)) * 180/PI_);
      }
      if(turn == 1){
        if(ang_z < target_z) {
          if(fabs(ang_z - target_z)<180.0){ //ensuring the shortest path
            //ang_z += 1.0; //anticlockwise
            multiplier = +0.04;
            if(fabs(target_z - ang_z) > 0.01){
              if((trans_z = (fabs(target_z-ang_z) * multiplier))> 0.05){
                trans_z = std::min(0.5, trans_z); //get the correct rotation
                printf("Too MUCH Trans_z, trans_z = %f\n", trans_z);
              }
              else{
                trans_z = fabs(target_z-ang_z) * multiplier;
                printf("trans_z = %f\n", trans_z);
              }

            }
            else{
              trans_z = 0.00;
              turn = 0;
            }
          }
          else{
            //ang_z -= 1.0; //clockwise
            multiplier = -0.04;
            if(fabs(target_z - ang_z) > 0.01){
              if((trans_z =(fabs(target_z-ang_z) * fabs(multiplier))) > 0.05){
                trans_z = std::max(-0.5, -trans_z); //get the correct rotation
                printf("trans_z = %f\n", trans_z);
              }
              else{
                trans_z = fabs(target_z-ang_z) * multiplier;
                printf("trans_z = %f\n", trans_z);
              }
            }
            else{
              trans_z = 0.0;
              turn = 0;
            }
          }
        }

        else {
          if(fabs(ang_z - target_z)<180.0){
            //ang_z -= 1.0; //clockwise
            multiplier = -0.04;
            if(fabs(target_z - ang_z) > 0.01){
              if((trans_z =(fabs(target_z-ang_z) * fabs(multiplier))) > 0.05){
                trans_z = std::max(-0.5, -trans_z); //get the correct rotation
                printf("trans_z = %f\n", trans_z);
              }
              else{
                trans_z = fabs(target_z-ang_z) * multiplier;
                printf("trans_z = %f\n", trans_z);
              }
            }
            else{
              trans_z = 0.0;
              turn = 0;

            }
          } //ensuring the shortest path

          else{ //ensuring the shortest path
            multiplier = +0.04;
            if(fabs(target_z - ang_z) > 0.01){
              if((trans_z = (fabs(target_z-ang_z) * multiplier)) > 0.05){
                trans_z = std::min(0.5, trans_z); //get the correct rotation
                printf("trans_z = %f\n", trans_z);
              }
              else{
                trans_z = fabs(target_z-ang_z) * multiplier;
                printf("trans_z = %f\n", trans_z);
              }
            }
            else{
              trans_z = 0.0;
              turn = 0;

            }
          }
        }
      }
      else{
        if(turn == 0){
          std::cout << "ready to move" << '\n';
        }
      }
      if(turn == 0 && move_x == 1 && move_y == 0){
        if(fabs(target_x - pos_x) > 0.01){
          trans_x = 0.3;
        }
        else{
          trans_x = 0.0;
          move_x = 0;
          move_y = 0;
          is_moving = false;
        }
      }
      if(turn == 0 && move_y == 1 && move_x == 0){
        if(fabs(target_y - pos_y) > 0.01){
          trans_x = 0.3;
        }
        else{
          trans_x = 0.0;
          move_x = 0;
          move_y = 0;
          is_moving = false;
        }
      }
      if(move_y == 1 || move_x == 1){
        base_cmd.linear.x = trans_x;
      }
      if(turn == 1){
        base_cmd.angular.z = trans_z;
      }
      vel_pub.publish(base_cmd);
      std::cout<< std::setprecision(5) << std::fixed;
      //std::cout << poseMsg->header.stamp
      //  << " C:" << pos_x << "," << pos_y << "," << ori_z
      //  << " T:" << target_x << "," << target_y << "," << target_o
      //  << " M:" <<  trans_x << ", " << trans_z << std::endl;

    }
  };

  int main(int argc, char** argv){
    ros::init(argc, argv, "bot_navigator");
    ros::NodeHandle nh;
    BotController bn(nh);

    ros::spin();
    return 0;
  }
