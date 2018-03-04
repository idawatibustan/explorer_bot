#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
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
  int getId() { return this->id; }
  double getH() { return this->h; }
  double getF() {
    if (calculated) {
      return this->f;
    }
    else {
      this->f = this->g + this->h;
      this->calculated = true;
      return f;
    }
  }
  double setG(double g) {
    this->g = g;
    this->calculated = false;
  }
};

class Graph{
private:
  std::map<int, std::vector<int>> adj;
  int node_count;
  int edge_count;
public:
  Graph(int node_num){
    this->node_count=node_num;
    this->edge_count=0;
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
    std::cout << "removing " << a << "," << b << std::endl;
    edge_count--;
    adj[a].erase(std::remove(adj[a].begin(), adj[a].end(), b), adj[a].end());
    adj[b].erase(std::remove(adj[b].begin(), adj[b].end(), a), adj[b].end());
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
  int size;
  int id_goal, id_init, id_curr;
  Position goal, init, curr;
  Graph graph;
  bool is_generated;
  std::map<int, Node*> nodes;
  // std::pair<int, Node*> nodes_pair;
public:
  // std::vector<Node> nodes;
  Map(int size, const Position &goal, const Position &init)
  : goal(goal), init(init), curr(init), graph(size*size) {
    this->size = size;
    this->id_goal = size * this->goal.getX() + this->goal.getY();
    this->id_init = size * this->init.getX() + this->init.getY();
    this->id_curr = size * this->curr.getX() + this->curr.getY();
    this->is_generated = false;
    print();
  }
  Map()
  : goal(1,1), init(0,0), curr(0,0), graph(9) {
    this->size = 3;
    this->id_goal = size * this->goal.getX() + this->goal.getY();
    this->id_init = size * this->init.getX() + this->init.getY();
    this->id_curr = size * this->curr.getX() + this->curr.getY();
    this->is_generated = false;
    print();
  }
  void print() {
    std::cout << " Goal:" << goal.getX() << ", " << goal.getY()
    << " Init:" << init.getX() << ", " << init.getY()
    << " Curr:" << curr.getX() << ", " << curr.getY() << std::endl;
  }
  void generateGraph(){
    if (this->is_generated){
      return;
    }
    int id = 0;
    for(int i=0; i<this->size; i++){
      for(int j=0; j<this->size; j++){
        id = size*i + j;
        // std::cout << "n:" << size << " id:" << id << " i:" << i << " j:" << j << std::endl;
        Node* n = new Node(id, i, j, goal);
        nodes[id] = n;
        // nodes[id] = new Node(id, i, j, goal);
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
    this->is_generated = true;
  }
  std::map<int, Node*> getNodes() { return nodes; };
  Graph* getGraph() { return &graph; }
  void viewGraph() { this->graph.printGraph(); };
  void solveMap() { 
    std::cout << "goal[" << id_goal << "]" << std::endl;
    // get current node object
    double g_val = 0.0;
    Node* n_curr = this->nodes[id_curr];
    n_curr->setG(g_val);
    this->graph.getEdges(id_curr);
    g_val++;

    std::cout << std::setprecision(2) << std::fixed;
    double min_f = 1000.0;  
    Node* n_min = NULL;
    while(id_curr != id_goal) {
    // get neighbor & update f & g
      min_f = 1000.0;
      n_min = NULL;
      for(int &i: this->graph.getEdges(id_curr)){
        Node* n_temp = this->nodes[i];
        n_temp->setG(g_val);
        std::cout << "  node[" << i << "] h=" << n_temp->getH()
                  << " f=" << n_temp->getF() << std::endl;
        if(n_temp->getF() < min_f){
          min_f = n_temp->getF();
          n_min = n_temp;
        }
      }
      n_curr = n_min;
      this->id_curr = n_curr->getId();
      std::cout << "* node[" << n_curr->getId() << "] h=" << n_curr->getH()
                << " min_f=" << n_curr->getF() << std::endl << std::endl;
      g_val++;
    }
    // pick lowest f
    // set curr to f
    // stop when reach goal
  }
};

int main(int argc, char** argv)
{
  Position start(0,0);
  Position goal(4,4);
  double dist = start.getDistance(goal);
  Node n(1, 0, 0, goal);
  std::cout << dist << " " << n.getH() << std::endl;
  Node n2(2, start, goal);
  std::cout << dist << " " << n2.getH() << std::endl;

  // Map m1;

  std::cout << "testing graph m2..." << std::endl;
  Map m2(9, goal, start);
  std::cout << "generating graph..." << std::endl;
  m2.generateGraph();
  std::cout << "printing graph..." << std::endl;
  // m2.viewGraph();
  
  std::cout << "printing nodes..." << std::endl;
  std::map<int, Node*> nodes = m2.getNodes();
  for(int i=0; i< m2.getGraph()->getNodeCount(); i++){ 
    Node* n = nodes[i];
    std::cout << "n[" << i << "] (" << n->getPosX() <<"," << n->getPosY() << ") d:" << n->getH() << std::endl;
  }

  std::cout << "solving map..." << std::endl;
  m2.solveMap();

  std::cout << "removing edge..." << std::endl;
  m2.getGraph()->removeEdge(0,1);
  // m2.viewGraph();

  std::cout << "solving map..." << std::endl;
  m2.solveMap();

}
