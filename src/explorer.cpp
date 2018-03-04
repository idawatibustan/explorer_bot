#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

class Position;
class Node;
// class Edge;

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
  int id;
  Position p;
  double g, h, f;
  bool calculated=false;
public:
  Node(int id, int x, int y, const Position &goal) : p(x, y) {
    this->id = id;
    this->h = this->p.getDistance(goal);
  }
  Node(int id, const Position &p, const Position &goal) : p(p) {
    this->id = id;
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

// class Edge{
// private:
//   bool checked;
//   Node node_a, node_b;
// public:
//   std::vector<Node> nodes;
//   Edge(const Node &a, const Node &b)
//   : node_a(a), node_b(b) {
//     this->checked = false;
//   }
//   Node getA() { return node_a; }
//   Node getB() { return node_b; }
//   bool get_checked() { return checked; }
//   void set_checked() { checked = true; }
// };

class Graph{
private:
  std::map<int, std::vector<int>> adj;
  int node_count;
  int edge_count;
public:
  Graph(int node_num){
    node_count=node_num;
    edge_count=0;
  };
  bool isEdgeValid(int a, int b){
    bool isBinA = (std::find(adj[a].begin(), adj[a].end(), b) != adj[a].end());
    bool isAinB = (std::find(adj[b].begin(), adj[b].end(), a) != adj[b].end());
    return !isBinA && !isAinB;
  }
  void addEdge(int a, int b){
    if(isEdgeValid(a, b)) {
      edge_count++;
      adj[a].push_back(b);
      adj[b].push_back(a);
    }
  }
  bool removeEdge(int a, int b){
    edge_count--;
    adj[a].erase(std::remove(adj[a].begin(), adj[a].end(), b), adj[a].end());
    adj[b].erase(std::remove(adj[b].begin(), adj[b].end(), a), adj[b].end());
  }
  std::vector<int> getEdges(int n){
    return adj[n];
  }
  void printNeighbors(int n) {
    std::cout << '[' << n << "] ";
    for(std::vector<int>::const_iterator j = adj[n].begin(); j!= adj[n].end(); ++j){
      std::cout << *j << ' ';
    }
    std::cout << std::endl;
  }
  void printGraph() {
    for(int i = 0; i < node_count; i++){
      printNeighbors(i);
    }
  }
};

class Map{
private:
  int size;
  Position goal, init, curr;
  Graph graph;
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
        int id = size*i + j;
        nodes.push_back(Node(id, i, j, goal));
      }
    }
  }

  void generateGraph(){
    int id = 0;
    for(int i=0; i<this->size; i++){
      for(int j=0; j<this->size; j++){
        id = size*i + j;
        // std::cout << "n:" << size << " id:" << id << " i:" << i << " j:" << j << std::endl;
        nodes.push_back(Node(id, i, j, goal));
        if (j > 0) {
          this->graph.addEdge(id-1, id);
        }
        if (j < size-1) {
          this->graph.addEdge(id, id+1);
        }
        if (i > 0) {
          this->graph.addEdge(id-size, id);
        }
        if (i < size-1) {
          this->graph.addEdge(id, id+size);
        }
        // this->graph.printGraph();
      }
    }
  }

  std::vector<Node> getNodes() { return nodes; };
  
  void viewGraph() { this->graph.printGraph(); };
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
  Map map;

public:
  Explorer(ros::NodeHandle &nh) : map() {
    goal_reached = false;

    map.generateMap();

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
      goal_reached = true;
    } else {
      status.data = "nope";
      goal_reached = false;
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
