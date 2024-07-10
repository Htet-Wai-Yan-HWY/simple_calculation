#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

using namespace std;

class Node {
public:
  int x, y;
  float g = INFINITY;
  float h = 0;
  Node *parent = nullptr;

  Node(int x = 0, int y = 0) : x(x), y(y) {}

  bool operator<(const Node &other) const {
    return (g + h) > (other.g + other.h); // Use > for min-heap priority_queue
  }

  bool operator==(const Node &other) const {
    return x == other.x && y == other.y;
  }
};

// Custom hash function for Node to use in unordered_set
struct NodeHash {
  std::size_t operator()(const Node &n) const {
    return std::hash<int>()(n.x) ^ std::hash<int>()(n.y);
  }
};

float heuristic(const Node &node, const Node &goal) {
  return abs(node.x - goal.x) + abs(node.y - goal.y);
}

vector<Node> get_neighbors(const Node &node, const vector<vector<int>> &grid) {
  vector<Node> neighbors;
  int directions[4][2] = {
      {0, 1}, {0, -1}, {1, 0}, {-1, 0}}; // 4-directional movement

  for (const auto &dir : directions) {
    int nx = node.x + dir[0];
    int ny = node.y + dir[1];

    if (nx >= 0 && nx < grid.size() && ny >= 0 && ny < grid[0].size() &&
        grid[nx][ny] == 0) {
      neighbors.push_back({nx, ny});
    }
  }

  return neighbors;
}

vector<pair<int, int>> reconstruct_path(const Node *node) {
  vector<pair<int, int>> path;
  const Node *current = node;

  while (current != nullptr) {
    path.push_back({current->x, current->y});
    current = current->parent;
  }

  reverse(path.begin(), path.end());
  return path;
}

vector<pair<int, int>> a_star(const vector<vector<int>> &grid, Node start,
                              const Node &goal) {
  priority_queue<Node> open_set;
  unordered_set<Node, NodeHash> closed_set;

  start.g = 0;
  start.h = heuristic(start, goal);
  open_set.push(start);

  while (!open_set.empty()) {
    Node current = open_set.top();
    open_set.pop();

    if (current == goal) {
      return reconstruct_path(&current);
    }

    closed_set.insert(current);

    for (auto neighbor : get_neighbors(current, grid)) {
      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }

      float tentative_g =
          current.g + 1; // Assuming cost of moving between adjacent nodes is 1

      if (tentative_g < neighbor.g) {
        neighbor.g = tentative_g;
        neighbor.h = heuristic(neighbor, goal);
        neighbor.parent = new Node(current); // Use new to store parent

        open_set.push(neighbor);
      }
    }
  }

  return {}; // No path found
}

void draw_path(vector<vector<int>> &grid, const vector<pair<int, int>> &path) {
  for (const auto &point : path) {
    grid[point.first][point.second] = -1; // Marking the path
  }
}

int main() {
  vector<vector<int>> grid = {{0, 0, 0, 0, 0},
                              {0, 1, 1, 0, 0},
                              {0, 0, 0, 0, 0},
                              {0, 0, 1, 1, 0},
                              {0, 0, 0, 0, 0}};

  cout << "Initial grid:" << endl;
  for (const auto &row : grid) {
    for (int cell : row) {
      cout << cell << " ";
    }
    cout << endl;
  }

  Node start_node{0, 0};
  Node goal_node{3, 2}; // Adjust goal position to be within the grid

  vector<pair<int, int>> path = a_star(grid, start_node, goal_node);

  if (!path.empty()) {
    cout << "\nPath found:" << endl;
    draw_path(grid, path);

    for (const auto &row : grid) {
      for (int cell : row) {
        if (cell == -1) {
          cout << "* ";
        } else {
          cout << cell << " ";
        }
      }
      cout << endl;
    }
  } else {
    cout << "Path not found." << endl;
  }

  return 0;
}
