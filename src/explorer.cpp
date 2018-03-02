#include <cmath>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <cmath>
#include <vector>

class Position{
private:
  int x, y;
public:
  Position(int x, int y){
    this->x = x; this->y = y;
  }
  int getX() { return x; }
  int getY() { return y; }
  double getDistance(Position other){
    double x = (double) ( this->x - other.getX() );
    double y = (double) ( this->y - other.getY() );
    return sqrt( pow(x, 2) + pow(y, 2) );
  }
};

class Node
{
private:
  Position p;
  double g, h, f;
  bool calculated=false;
public:
  Node(int x, int y, const Position &goal) : p(x, y) {
    this->h = this->p.getDistance(goal);
  }
  Node(const Position &p, const Position &goal) : p(p) {
    this->h = this->p.getDistance(goal);
  }
  double getPosX() { return p.getX(); }
  double getPosY() { return p.getY(); }
  double getH() { return h; }
  double getF() {
    if (calculated)
    return g+h;
    else
    return 0.0;
  }
  double setG(double g) {
    this->g = g;
  }
};

class Map{
private:
  int size;
  Position goal, init, curr;
public:
  std::vector<Node> nodes;
  Map(int size, const Position &goal, const Position &init)
  : goal(goal), init(init), curr(init) {
    this->size = size;
    print();
  }
  Map()
  : goal(4,4), init(0,0), curr(0,0) {
    this->size = 9;
    print();
  }
  void print() {
    std::cout << " Goal:" << goal.getX() << ", " << goal.getY()
    << " Init:" << init.getX() << ", " << init.getY()
    << " Curr:" << curr.getX() << ", " << curr.getY() << std::endl;
  }
  void generateMap() {
    std::cout << "gen " << size << std::endl;
    for(int i=0; i<size; i++){
      for(int j=0; j<size; j++){
        // std::cout << "n " << (i+1)*(j+1) << std::endl;
        nodes.push_back(Node(i, j, goal));
      }
    }
  }

  std::vector<Node> getNodes() { return nodes; };
};

class Explorer{
private:
  ros::Subscriber pos_sub, wall_sub;
  ros::Publisher expl_pub;
  std_msgs::String status;
  double pos_x, pos_y, ori_z, ang_z;
  double goal_x, goal_y;
  double d_x, d_y;

  bool goal_reached = false;
  bool wall_front, wf_left, wf_front, wf_right;

public:
  Explorer(ros::NodeHandle &nh){
    goal_reached = false;

    goal_x = 4.0;
    goal_y = 4.0;

    pos_sub = nh.subscribe("/odom",1,&Explorer::odom_callback, this);
    wall_sub = nh.subscribe("/wall_scan",1,&Explorer::wall_callback, this);
    expl_pub = nh.advertise<std_msgs::String>("/explorer_status",1);
  }
  void odom_callback( const nav_msgs::OdometryConstPtr& poseMsg ) {
    pos_x = poseMsg->pose.pose.position.x;
    pos_y = poseMsg->pose.pose.position.y;
    ori_z = poseMsg->pose.pose.orientation.z;
    ang_z = ori_z*2.19;

    d_x = std::abs(pos_x - goal_x);
    d_y = std::abs(pos_y - goal_y);
    if ( d_x < 0.2 && d_y < 0.2 ) {
      status.data = "goal";
    } else {
      status.data = "nope";
    }

    expl_pub.publish(status);
    // std::cout<< std::setprecision(2) << std::fixed;
    // std::cout << poseMsg->header.stamp
    //           << " Curr:" << pos_x << ", " << pos_y
    //           << " Trgt:" << goal_x << ", " << goal_y
    //           << " Dist:" << d_x << ", " << d_y << std::endl;
  }
  void wall_callback( const std_msgs::StringConstPtr& msg ) {
    std::string res = msg->data.c_str();
    wall_front = (res[0] == '1');
    wf_left = (res[1] == '1');
    wf_front = (res[2] == '1');
    wf_right = (res[3] == '1');
    std::cout << wall_front << wf_left << wf_front << wf_right << std::endl;
  }
  bool is_goal_reached() { return goal_reached; }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "explorer");
  ros::NodeHandle nh;
  Explorer ex(nh);

  ros::Rate r(10);
  while (ros::ok()) {
    if (!ex.is_goal_reached())
      ROS_INFO("trying");
    ROS_INFO("again");
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
