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

int main(int argc, char** argv)
{
  Position start(0,0);
  Position goal(4,4);
  double dist = start.getDistance(goal);
  Node n(0, 0, goal);
  std::cout << dist << " " << n.getH() << std::endl;
  Node n2(start, goal);
  std::cout << dist << " " << n2.getH() << std::endl;
  Map m1;
  Map m2(9, goal, start);
  m1.generateMap();
  // for (Node& n : m1.getNodes()){
  //   std::cout << "n(" << n.getPosX() <<"," << n.getPosY() << ") d:" << n.getH() << std::endl;
  // }
}
