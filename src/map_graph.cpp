#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

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
public:
  Node(int id, int x, int y, const Position &goal) : p_(x, y) {
    this->id_ = id;
    this->h_ = this->p_.getDistance(goal);
    this->calculated = false;
  }
  Node(int id, const Position &p, const Position &goal) : p_(p) {
    this->id_ = id;
    this->h_ = this->p_.getDistance(goal);
    this->calculated = false;
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
    for(int i=0; i<size*size; i++) {
      this->closed_nodes[i] = 0;
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
    for(int i=0; i<81; i++) {
      this->closed_nodes[i] = 0;
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
        Node* n = new Node(id, i, j, this->goal_);
        // std::cout << "n:" << this->size_ << " id:" << id << " i:" << i << " j:" << j
        //           << " g=" << n->getH() << std::endl;
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
      std::cout << std::endl << std::endl;
    }
  }
  int getNeighborId(int id, int direction) {
    // dir: [0, 1, 2, 3] = [n, e, s, w]
    switch(direction) {
      case 0 : return id+this->size_;
      case 1 : return id+1;
      case 2 : return id-this->size_;
      case 3 : return id-1;
    }
    return -1;
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
  void forceUpdateEdge(int id, int ori, bool wall_front) {
    // ori: [0, 1, 2, 3] = [n, e, s, w]
    std::cout << "FORCE UPDATE " << id << ">" << ori
              << " front:" << wall_front << std::endl;
    int id_neighbor = this->getNeighborId(id, ori);
    if(wall_front) {
      this->graph.removeEdge(id, id_neighbor);
    } else {
      this->graph.addEdge(id, id_neighbor);
    }
  }
  bool updateEdge(int id, int ori, bool wall_front) {
    // ori: [0, 1, 2, 3] = [n, e, s, w]
    std::cout << "WALL self " << id << ">" << ori
              << " front:" << wall_front << std::endl;
    bool wall_change = false;
    if(wall_front) {
      int id_neighbor = this->getNeighborId(id, ori);
      wall_change = this->graph.removeEdge(id, id_neighbor);
    }
    if(wall_change) {
      std::cout << "updating edge, wall_changed" << std::endl;
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
    if (min_f == 1000.0 || n_min == NULL){
      std::cout << "Next node not found!" << std::endl;
      return -2;
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
};

int main()
{
  Position start(0,0);
  Position goal(4,4);
  double dist = start.getDistance(goal);
  Node n1(1, 0, 0, goal);
  std::cout << dist << " " << n1.getH() << std::endl;
  Node n2(2, start, goal);
  std::cout << dist << " " << n2.getH() << std::endl;

  // Map m1;

  std::cout << "testing graph m2..." << std::endl;
  // Map m2(9, goal, start);
  Map m2(9, Position(4,4), Position(0,0));
  std::cout << "generating graph..." << std::endl;
  m2.generateGraph();
  std::cout << "printing graph..." << std::endl;
  m2.viewGraph();
  
  std::cout << "printing nodes..." << std::endl;
  std::map<int, Node*> nodes = m2.getNodes();
  for(int i=0; i< m2.getGraph()->getNodeCount(); i++){ 
    Node* n = nodes[i];
    std::cout << "n[" << i << "] (" << n->getPosX() <<"," << n->getPosY() << ") d:" << n->getH() << std::endl;
  }

  std::cout << "removing edge..." << std::endl;
  m2.removeEast(0);
  m2.removeWest(10);
  m2.removeSouth(30);
  m2.removeNorth(22);
  // m2.removeWest(22);
  m2.updateEdge(21, 0, true);
  m2.updateEdge(21, 1, true);
  m2.updateEdge(21, 2, true);
  m2.updateEdge(21, 3, true);
  // m2.updateEdge(20, 1, true);
  m2.updateEdge(22, 0, true, true, false);
  // m2.printAdj();

  std::cout << "checking neighbors code... " << std::endl;
  std::cout << "north 10 -> " << m2.getNeighborId(10, 0)
            << " east 10 -> " << m2.getNeighborId(10, 1)
            << " south 10 -> " << m2.getNeighborId(10, 2)
            << " west 10 -> " << m2.getNeighborId(10, 3) << std::endl;

  std::cout << "solving map..." << std::endl;
  m2.solveMap();
  m2.viewGraph();

}
