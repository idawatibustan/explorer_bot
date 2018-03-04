#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
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
    bool resA = (std::find(adj[a].begin(), adj[a].end(), b) != adj[a].end());
    bool resB = (std::find(adj[b].begin(), adj[b].end(), a) != adj[b].end());
    return resA && resB;
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
  : goal(goal), init(init), curr(init), graph(size*size) {
    this->size = size;
    print();
  }
  Map()
  : goal(1,1), init(0,0), curr(0,0), graph(9) {
    this->size = 3;
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
        std::cout << "n " << id << ' ' << i << ' ' << j << std::endl;
        nodes.push_back(Node(id, i, j, goal));
        if (j > 0) {
          this->graph.addEdge(id-1, id);
        }
        if (j < size) {
          this->graph.addEdge(id, id+1);
        }
        if (i > 0) {
          this->graph.addEdge(id-size, id);
        }
        if (i < size) {
          this->graph.addEdge(id, id+size);
        }
        this->graph.printGraph();
      }
    }
  }

  std::vector<Node> getNodes() { return nodes; };
  void viewGraph() { this->graph.printGraph(); };
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
  Map m1;
  Map m2(9, goal, start);
  m1.generateMap();
  std::cout << "generating graph..." << std::endl;
  m1.generateGraph();
  std::cout << "printing graph..." << std::endl;
  m1.viewGraph();
  // for (Node& n : m1.getNodes()){
  //   std::cout << "n(" << n.getPosX() <<"," << n.getPosY() << ") d:" << n.getH() << std::endl;
  // }
}
