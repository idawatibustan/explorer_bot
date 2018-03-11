#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stack>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <explorer_bot/MoveGoal.h>

class Position{
private:
  int x_, y_;
public:
  Position(int x, int y){
    this->x_ = x; this->y_ = y;
  }
  int getX() { return this->x_; }
  int getY() { return this->y_; }
  double getDistance(Position other){
    double dx = this->x_ - other.getX();
    double dy = this->y_ - other.getY();
    return sqrt( pow(dx, 2) + pow(dy, 2) );
  }
};

class Node
{
private:
  int id_;
  Position p_;
  double g_, h_, f_;
  bool calculated;
  bool visited;
public:
  Node(int id, int x, int y, const Position &goal) : p_(x, y) {
    this->id_ = id;
    this->h_ = this->p_.getDistance(goal);
    this->calculated = false;
    this->visited = false;
  }
  Node(int id, const Position &p, const Position &goal) : p_(p) {
    this->id_ = id;
    this->h_ = this->p_.getDistance(goal);
    this->calculated = false;
    this->visited = false;
  }
  double getPosX() { return this->p_.getX(); }
  double getPosY() { return this->p_.getY(); }
  int getId() { return this->id_; }
  double getH() { return this->h_; }
  double getF() {
    if (this->calculated) {
      return this->f_;
    }
    else {
      this->f_ = this->g_ + this->h_;
      this->calculated = true;
      return this->f_;
    }
  }
  double getG() { return this->g_; }
  void setG(double g) {
    this->g_ = g;
    this->calculated = false;
  }
  bool is_visited() { return this->visited; }
  void mark_visited() {
    this->visited = true;
  }
};

class Graph{
private:
  std::map< int, std::vector<int> > adj;
  int node_count;
  int edge_count;
public:
  Graph(int node_num){
    this->node_count=node_num;
    this->edge_count=0;
  }
  bool doesEdgeExist(int a, int b){
    bool b_in_a = (std::find(adj[a].begin(), adj[a].end(), b) != adj[a].end());
    bool a_in_b = (std::find(adj[b].begin(), adj[b].end(), a) != adj[b].end());
    // std::cout << "check if exist b_in_a: " << b_in_a
    //           << " a_in_b: " << a_in_b << std::endl;
    return b_in_a && a_in_b;
  }
  void addEdge(int a, int b){
    if(!doesEdgeExist(a, b)) {
      edge_count++;
      adj[a].push_back(b);
      adj[b].push_back(a);
    }
  }
  bool removeEdge(int a, int b){
    if(doesEdgeExist(a, b)) {
      std::cout << "removing " << a << "," << b << std::endl;
      edge_count--;
      adj[a].erase(std::remove(adj[a].begin(), adj[a].end(), b), adj[a].end());
      adj[b].erase(std::remove(adj[b].begin(), adj[b].end(), a), adj[b].end());
      return true;
    }
    return false;
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
    std::cout << "nodes: " << node_count
    << ", edges:" << edge_count << std::endl;
  }
  std::vector<int> getEdges(int n) { return adj[n]; }
  int getNodeCount() { return node_count; }
  int getEdgeCount() { return edge_count; }
};

class Map{
private:
  int size_;
  int id_goal, id_init, id_curr;
  Position goal_, init_, curr_;
  Graph graph;
  bool is_generated;
  std::map<int, Node*> nodes;
  std::vector<int> path;

  int* closed_nodes;
  // int* open_nodes;

public:
  Map(int size, const Position &goal, const Position &init)
  : goal_(goal), init_(init), curr_(init), graph(size*size) {
    this->size_ = size;
    this->id_goal = size * this->goal_.getX() + this->goal_.getY();
    this->id_init = size * this->init_.getX() + this->init_.getY();
    this->id_curr = size * this->curr_.getX() + this->curr_.getY();
    this->path.push_back(this->id_init);
    this->is_generated = false;
    this->closed_nodes = new int[size*size];
    // this->open_nodes = new int[size*size];

    for(int i=0; i<size*size; i++) {
      this->closed_nodes[i] = 0;
      // this->open_nodes[i] = 0;
    }
    print();
  }
  Map()
  : goal_(4,4), init_(0,0), curr_(0,0), graph(81) {
    this->size_ = 9;
    this->id_goal = this->size_ * this->goal_.getX() + this->goal_.getY();
    this->id_init = this->size_ * this->init_.getX() + this->init_.getY();
    this->id_curr = this->size_ * this->curr_.getX() + this->curr_.getY();
    this->path.push_back(this->id_init);
    this->is_generated = false;
    this->closed_nodes = new int[81];
    // this->open_nodes = new int[81];

    for(int i=0; i<81; i++) {
      this->closed_nodes[i] = 0;
      // this->open_nodes[i] = 0;
    }
    print();
  }
  void print() {
    std::cout << " Goal:" << this->goal_.getX() << ", " << this->goal_.getY()
    << " Init:" << this->init_.getX() << ", " << this->init_.getY()
    << " Curr:" << this->curr_.getX() << ", " << this->curr_.getY() << std::endl;
  }
  void generateGraph(){
    if (this->is_generated){
      return;
    }
    int id = 0;
    for(int i=0; i<this->size_; i++){
      for(int j=0; j<this->size_; j++){
        id = this->size_ * i + j;
        // std::cout << "n:" << size << " id:" << id << " i:" << i << " j:" << j << std::endl;
        Node* n = new Node(id, i, j, this->goal_);
        nodes[id] = n;
        if (j > 0) {
          this->graph.addEdge(id-1, id);
        }
        if (j < this->size_-1) {
          this->graph.addEdge(id, id+1);
        }
        if (i > 0) {
          this->graph.addEdge(id-this->size_, id);
        }
        if (i < this->size_-1) {
          this->graph.addEdge(id, id+this->size_);
        }
        // this->graph.printGraph();
      }
    }
    nodes[id_init]->setG(0.0);
    this->is_generated = true;
  }
  std::map<int, Node*> getNodes() { return nodes; }
  Graph* getGraph() { return &graph; }
  void printAdj() { this->graph.printGraph(); }
  void viewGraph() { 
    for(int i=this->size_-1; i>=0; i--){
      for(int j=0; j<this->size_; j++){
        int id = this->size_ * i + j;
        if (std::find(path.begin(), path.end(), id) != path.end()){
          std::cout << "[ *] ";
        } else {
          std::cout << '[' << std::setw(2) << id << "] ";
        }
      }
      std::cout << std::endl;
      std::cout << std::endl;
    }
  }
  bool removeNorth(int id) {
    int id_north = id+this->size_;
    std::cout << "remove north: [" << id << ", " << id_north << "]" << std::endl; 
    if (id_north >= this->graph.getNodeCount()) {
      return false;
    }
    return this->graph.removeEdge(id, id_north);
  }
  bool removeEast(int id) {
    int id_east = id+1;
    std::cout << "remove east: [" << id << ", " << id_east << "]" << std::endl; 
    if (id_east >= this->graph.getNodeCount()) {
      return false;
    }
    return this->graph.removeEdge(id, id_east);
  }
  bool removeSouth(int id) {
    int id_south = id-this->size_;
    std::cout << "remove south: [" << id << ", " << id_south << "]" << std::endl; 
    if (id_south >= this->graph.getNodeCount()) {
      return false;
    }
    return this->graph.removeEdge(id, id_south);
  }
  bool removeWest(int id) {
    int id_west = id-1;
    std::cout << "remove west: [" << id << ", " << id_west << "]" << std::endl; 
    if (id_west >= this->graph.getNodeCount()) {
      return false;
    }
    return this->graph.removeEdge(id, id_west);
  }
  bool updateEdge(int id, int ori, bool wall_front) {
    // ori: [0, 1, 2, 3] = [n, e, s, w]
    std::cout << "WALL self " << id << ">" << ori
              << " front:" << wall_front << std::endl;
    bool wall_change = false;
    if(wall_front) {
      switch(ori) {
        case 0 : wall_change = this->removeNorth(id); break;
        case 1 : wall_change = this->removeEast(id); break;
        case 2 : wall_change = this->removeSouth(id); break;
        case 3 : wall_change = this->removeWest(id); break;
      }
    }
    if(wall_change) {
      std::cout << "updating edge, wall_changed" << std::endl;
    // } else {
    //   std::cout << "no change" << std::endl;
    }
    return wall_change;
  }
  bool updateEdge(int id, int ori, bool wall_left, bool wall_front, bool wall_right){
    std::cout << " --> WALL front " << id << ">" << ori
              << " walls:" << wall_left << wall_front << wall_right << std::endl;
    bool left_change=false, front_change=false, right_change=false;
    if(wall_left) {
      switch(ori) {
        case 0 : left_change = this->removeWest(id); break;
        case 1 : left_change = this->removeNorth(id); break;
        case 2 : left_change = this->removeEast(id); break;
        case 3 : left_change = this->removeSouth(id); break;
      }
    }
    if(wall_front) {
      switch(ori) {
        case 0 : front_change = this->removeNorth(id); break;
        case 1 : front_change = this->removeEast(id); break;
        case 2 : front_change = this->removeSouth(id); break;
        case 3 : front_change = this->removeWest(id); break;
      }
    }
    if(wall_right) {
      switch(ori) {
        case 0 : right_change = this->removeEast(id); break;
        case 1 : right_change = this->removeSouth(id); break;
        case 2 : right_change = this->removeWest(id); break;
        case 3 : right_change = this->removeNorth(id); break;
      }
    }
    if(left_change || front_change || right_change){
      std::cout << "updating edge, wall_changed" << left_change << front_change << right_change << std::endl;
    // } else {
    //   std::cout << "no change" << std::endl;
    }
    return left_change || front_change || right_change;
  }
  int solveNextStep(int n) {
    // get current node g(n) + 1 for next step
    std::cout << "solving node: " << n << std::endl;
    double g_n = this->nodes[n]->getG() + 1;
    // initialize min value
    double min_f = 1000.0;
    Node* n_min = NULL;

    std::cout << std::setprecision(2) << std::fixed;
    // iterate through the node neighbors
    for(int &i: this->graph.getEdges(n)){
      Node* n_temp = this->nodes[i];
      // update actual cost g(n)
      n_temp->setG(g_n);
      std::cout << "  node[" << std::setw(2) << i
      << "] h=" << std::setw(5) << n_temp->getH()
      << " f=" << std::setw(5) << n_temp->getF()
      << " c?" << closed_nodes[n_temp->getId()] << std::endl;
      // if lower than min, update min f(n) & index
      if(n_temp->getF() < min_f && closed_nodes[n_temp->getId()] == 0){
        min_f = n_temp->getF();
        n_min = n_temp;
      }
    }
    std::cout << "* node[" << std::setw(2) << n_min->getId()
    << "] h(n)=" << std::setw(5) << n_min->getH()
    << " min f(n)=" << std::setw(5) << n_min->getF()
    << std::endl << std::endl;
    // return node index with the lowest f(n)
    this->closed_nodes[n_min->getId()] = 1;
    this->path.push_back(n_min->getId());
    return n_min->getId();
  }
  void solveMap() {
    std::cout << "goal[" << this->id_goal << "]" << std::endl;
    // loop until current node is goal node
    while(this->id_curr != this->id_goal) {
      this->id_curr = this->solveNextStep(this->id_curr);
      // add next node to path
      path.push_back(this->id_curr);
    }
  }
  int peekNextPath(int n){
    bool getNext = false;
    for(int &j: this->path){
      if(getNext == true){
        return j;
      }
      if (j == n){
        getNext = true;
      }
    }
    return -1;
  }
  explorer_bot::MoveGoal getNodeGoal(int n) {
    explorer_bot::MoveGoal goal;
    goal.request.goal.x = this->nodes[n]->getPosX();
    goal.request.goal.y = this->nodes[n]->getPosY();
    goal.request.goal.theta = 0;
    return goal;
  }
};

class Explorer{
private:
  ros::Subscriber pos_sub, wall_sub, mov_sub;
  ros::Publisher expl_pub;
  std_msgs::String status;

  ros::ServiceClient turn_north, turn_east, turn_south, turn_west;
  ros::ServiceClient move_north, move_east, move_south, move_west;
  std_srvs::Empty esrv;
  explorer_bot::MoveGoal goal;

  double pos_x, pos_y, ori_z, ang_z;
  double roll, pitch, yaw;
  double goal_x, goal_y;
  double d_x, d_y;

  int init_count;
  bool init_completed, goal_reached;
  bool wall_front, wf_left, wf_front, wf_right;
  bool moving_flag;
  int map_size_;
  Map map;

public:
  Explorer(ros::NodeHandle &nh) : map() {
    this->init_completed = false;
    this->goal_reached = false;
    this->init_count = 0;

    map.generateGraph();
    map.viewGraph();
    map.printAdj();

    goal_x = 4.0;
    goal_y = 4.0;
    map_size_ = 9;

    moving_flag = false;

    wall_front = false;
    wf_left = false;
    wf_front = false;
    wf_right = false;

    pos_sub = nh.subscribe("/odom",1,&Explorer::odom_callback, this);
    wall_sub = nh.subscribe("/wall_scan",1,&Explorer::wall_callback, this);
    mov_sub = nh.subscribe("/is_moving",1,&Explorer::mov_callback, this);
    expl_pub = nh.advertise<std_msgs::String>("/explorer_status",1);

    turn_north = nh.serviceClient<std_srvs::Empty>("/turn_north");
    turn_east = nh.serviceClient<std_srvs::Empty>("/turn_east");
    turn_south = nh.serviceClient<std_srvs::Empty>("/turn_south");
    turn_west = nh.serviceClient<std_srvs::Empty>("/turn_west");

    move_north = nh.serviceClient<explorer_bot::MoveGoal>("/move_north");
    move_east = nh.serviceClient<explorer_bot::MoveGoal>("/move_east");
    move_south = nh.serviceClient<explorer_bot::MoveGoal>("/move_south");
    move_west = nh.serviceClient<explorer_bot::MoveGoal>("/move_west");
  }
  void odom_callback( const nav_msgs::OdometryConstPtr& poseMsg ) {
    pos_x = poseMsg->pose.pose.position.x;
    pos_y = -poseMsg->pose.pose.position.y;
    ori_z = poseMsg->pose.pose.orientation.z;
    ang_z = ori_z*2.19;

    tf::Quaternion q(poseMsg->pose.pose.orientation.x,
      poseMsg->pose.pose.orientation.y,
      poseMsg->pose.pose.orientation.z,
      poseMsg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(this->roll, this->pitch, this->yaw);

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
    // std::cout << wall_front << wf_left << wf_front << wf_right << std::endl;
  }
  void mov_callback( const std_msgs::BoolConstPtr& movMsg ) {
    this->moving_flag = movMsg->data;
  }
  bool is_init_completed() { return this->init_completed; }
  bool is_goal_reached() { return this->goal_reached; }
  void update_wall(int map_curr_id, int ori) {
    bool is_updated = false;
    is_updated = map.updateEdge(map_curr_id, ori, this->wall_front);
    if(!this->wall_front){
      // TODO: get orientation before deciding which to update
      int map_neighbor_id;
      switch(ori) {
        case 0 : map_neighbor_id = map_curr_id+this->map_size_; break;
        case 1 : map_neighbor_id = map_curr_id+1; break;
        case 2 : map_neighbor_id = map_curr_id-this->map_size_; break;
        case 3 : map_neighbor_id = map_curr_id+1; break;
      }
      is_updated = map.updateEdge(map_neighbor_id, ori, this->wf_left, this->wf_front, this->wf_right) || is_updated;
    }
    if(is_updated){
      this->map.printAdj();
    }
  }
  void init_search() {
    if(this->moving_flag)
      return;
    switch(init_count){
      case 0: ROS_INFO("Updating 1st wall");
              break;
      case 1: ROS_INFO("Updating 2nd wall");
              this->turn_east.call(esrv);
              break;
      case 2: ROS_INFO("Updating 3rd wall");
              this->turn_south.call(esrv);
              break;
      case 3: ROS_INFO("Updating 3rd wall");
              this->turn_west.call(esrv);
              break;
      case 4: ROS_INFO("Back to initial state");
              this->turn_north.call(esrv);
              break;
    }
    /*
    // declare initial position
    // update_wall;
    // if(east exists) turnEast, updateWall;
    // if(south exits) turn south, updateWall;
    // if(west exists) turn west, updateWall;
    */
  }
  void find_goal() {
    if(this->moving_flag)
      return;
    int x = round(pos_x);
    int y = round(pos_y);
    int z = round(yaw/1.5708);
    double d_z = std::abs( yaw/1.5708 - z );
    int map_curr_id = this->map_size_ * x + y;
    int map_curr_ori = int(round( -yaw/1.5708 ) + 4 ) % 4;

    std::cout << "   d_x:" << std::abs(pos_x - x)
              << " d_y:" << std::abs(pos_y - y)
              << " d_z:" << d_z << std::endl;
    std::cout << std::setprecision(3)
              << "   pos_x:" << pos_x << " x:" << x
              << " pos_y:" << pos_y << " y:" << y
              << " ang_z:" << ang_z
              << " yaw:" << yaw << std::endl;
    if ( std::abs(pos_x - x) < 0.1 && std::abs(pos_y - y) < 0.1 && d_z < 0.05 ) {
      // determine robot map_ori from yaw
      // ori: [0, 1, 2, 3] = [n, e, s, w]
      // std::cout << "   bot in the center, updating wall" << std::endl;
      std::cout << "**N[" << map_curr_id
                << "](" << x << "," << y
                << ") ori: " << map_curr_ori << std::endl;
      this->update_wall(map_curr_id, map_curr_ori);
      if( this->init_count < 6){
        switch (init_count) {
          case 0: this->init_count = (map_curr_ori == 0) ? 1 : 0; break;
          case 1: this->init_count = (map_curr_ori == 1) ? 4 : 1; break;
          case 2: this->init_count = (map_curr_ori == 2) ? 3 : 2; break;
          case 3: this->init_count = (map_curr_ori == 3) ? 4 : 3; break;
          case 4: this->init_count = (map_curr_ori == 0) ? 5 : 4;
                  break;
          case 5: this->init_count++;
                  this->init_completed = true;
                  this->map.solveNextStep(map_curr_id);
                  ROS_INFO("trying to solve next");
                  break;
        }
      }
    }

    int map_next_id = map_curr_id;
    if(this->init_completed == true) {
      map_next_id = this->map.peekNextPath(map_curr_id);
      std::cout << "next = " << map_next_id
                << "curr = " << map_curr_id << std::endl;
      if (map_next_id == -1) {
        this->map.solveNextStep(map_curr_id);
      }
    }
    if(map_curr_id != map_next_id){
      ROS_INFO("MOVE!!! %d -> %d", map_curr_id, map_next_id);
      int n = map_curr_id+this->map_size_;
      int e = map_curr_id+1;
      int s = map_curr_id-this->map_size_;
      int w = map_curr_id-1;
      this->goal = this->map.getNodeGoal(map_curr_id);
      ROS_INFO("moving now");
      if(!moving_flag) {
        if(map_next_id == n) {
          this->move_north.call(goal);
          moving_flag = true;
        } else if (map_next_id == e) {
          this->move_east.call(goal);
          moving_flag = true;
        } else if (map_next_id == e) {
          this->move_south.call(goal);
          moving_flag = true;
        } else if (map_next_id == e) {
          this->move_west.call(goal);
          moving_flag = true;
        }
      }
    } else {
      ROS_INFO("chillin!!! %d -> %d", map_curr_id, map_next_id);
    }

    return;
  }
  void close() {
    this->map.printAdj();
    this->map.viewGraph();
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "explorer");
  ros::NodeHandle nh;
  Explorer ex(nh);

  ros::Rate r(10);
  while (ros::ok()) {
    if (!ex.is_init_completed()) {
      ex.init_search();
    }
    if (!ex.is_goal_reached())
      ex.find_goal();
    ros::spinOnce();
    r.sleep();
  }
  ex.close();
  return 0;
}
