#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <map>
#include <set>
// Avoid using namespace std; it pollutes the global namespace and can lead to conflicts.
// using namespace std;
struct Node {
  int x, y;
  double f, g, h;
  Node* parent;
  Node(int x, int y) : x(x), y(y), f(0), g(0), h(0), parent(nullptr) {}
  bool operator==(const Node& other) const {
    return x == other.x && y == other.y;
  }
  bool operator<(const Node& other) const {
    return f < other.f;
  }
};
// Computes the Euclidean distance between two points as a heuristic.
double heuristic(int x1, int y1, int x2, int y2) {
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
// Finds the walkable neighbors of the current node in the grid.
std::vector<Node*> getNeighbors(Node* current, const std::vector<std::vector<int>>& map) {
  std::vector<Node*> neighbors;
  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;  // Skip the current node
      int newX = current->x + dx;
      int newY = current->y + dy;
      // Check if the neighbor is within grid bounds and is walkable (0 in the map indicates walkable).
      if (newX >= 0 && newX < map.size() && newY >= 0 && 
          newY < map[0].size() && map[newX][newY] == 0) {
        neighbors.push_back(new Node(newX, newY));
      }
    }
  }
  return neighbors;
}
// Reconstructs the path from the goal node to the start node by following parent pointers.
std::vector<Node*> reconstructPath(Node* current) {
  std::vector<Node*> path;
  while (current != nullptr) {
    path.push_back(current);
    current = current->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}
// Performs the A* search from a start node to a goal node on the given grid map.
std::vector<Node*> aStarSearch(const std::vector<std::vector<int>>& map, Node* start, Node* goal) {
  // Use a priority queue to store open nodes, sorted by their f-score (estimated total cost).
  std::priority_queue<Node*, std::vector<Node*>, std::less<>> openSet;
  // Use a set to efficiently check if a node is in the open set.
  std::set<std::pair<int, int>> openSetCheck;
  // Use a map to store the g-score (cost from start to current) for each node.
  std::map<std::pair<int, int>, double> gScore;
  start->h = heuristic(start->x, start->y, goal->x, goal->y);
  start->f = start->h;
  openSet.push(start);
  openSetCheck.insert({start->x, start->y});
  gScore[{start->x, start->y}] = 0;
  while (!openSet.empty()) {
    Node* current = openSet.top();
    openSet.pop();
    openSetCheck.erase({current->x, current->y});
    if (*current == *goal) { // Check if the current node is the goal node.
      return reconstructPath(current); 
    }
    // Iterate through the neighbors of the current node.
    for (Node* neighbor : getNeighbors(current, map)) {
      double tentativeGScore = gScore[{current->x, current->y}] + 
                               heuristic(current->x, current->y, neighbor->x, neighbor->y);
      if (!gScore.count({neighbor->x, neighbor->y}) || 
          tentativeGScore < gScore[{neighbor->x, neighbor->y}]) {
        // Update the neighbor's parent, g-score, h-score, and f-score.
        neighbor->parent = current;
        neighbor->g = tentativeGScore;
        neighbor->h = heuristic(neighbor->x, neighbor->y, goal->x, goal->y);
        neighbor->f = neighbor->g + neighbor->h;
        // If the neighbor is not in the open set, add it to the open set.
        if (!openSetCheck.count({neighbor->x, neighbor->y})) {
          openSet.push(neighbor);
          openSetCheck.insert({neighbor->x, neighbor->y});
          gScore[{neighbor->x, neighbor->y}] = tentativeGScore;
        }
      }
    }
  }
  // Return an empty path if the goal is unreachable.
  return {}; 
}
int main() {
  std::vector<std::vector<int>> map = {
      {0, 0, 0, 0, 1},
      {0, 1, 1, 0, 0},
      {0, 0, 0, 1, 0},
      {0, 1, 0, 0, 0},
      {0, 0, 0, 0, 0}
  };
  Node* start = new Node(0, 0);
  Node* goal = new Node(4, 4);
  std::vector<Node*> path = aStarSearch(map, start, goal);
  for (Node* node : path) {
    std::cout << "(" << node->x << ", " << node->y << ")" << std::endl;
  }
  // Clean up dynamically allocated memory to avoid memory leaks.
  for (Node* node : path) {
    delete node;
  }
  delete start;
  delete goal;
  return 0;
}