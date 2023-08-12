#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

using namespace std;

class Node {
public:
    mutable int x, y;
    mutable float g = INFINITY;
    mutable float h = 0;
    mutable Node* parent = nullptr;

    bool operator<(const Node& other) const {
        return (g + h) < (other.g + other.h);
    }
};

float heuristic(const Node& node, const Node& goal) {
    return abs(node.x - goal.x) + abs(node.y - goal.y);
}

vector<Node> get_neighbors(const Node& node, const vector<vector<int>>& grid) {
    vector<Node> neighbors;
    int directions[4][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}}; // 4-directional movement

    for (const auto& dir : directions) {
        int nx = node.x + dir[0];
        int ny = node.y + dir[1];

        if (nx >= 0 && nx < grid.size() && ny >= 0 && ny < grid[0].size() && grid[nx][ny] == 0) {
            neighbors.push_back({nx, ny});
        }
    }

    return neighbors;
}

vector<pair<int, int>> reconstruct_path(const Node* node) {
    vector<pair<int, int>> path;
    const Node* current = node;

    while (current != nullptr) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }

    reverse(path.begin(), path.end());
    return path;
}

vector<pair<int, int>> a_star(const vector<vector<int>>& grid, const Node& start, const Node& goal) {
    priority_queue<Node> open_set;
    start.g = 0;
    start.h = heuristic(start, goal);
    open_set.push(start);

    while (!open_set.empty()) {
        Node current = open_set.top();
        open_set.pop();

        if (current.x == goal.x && current.y == goal.y) {
            return reconstruct_path(&current);
        }

        for (const auto& neighbor : get_neighbors(current, grid)) {
            float tentative_g = current.g + 1; // Assuming cost of moving between adjacent nodes is 1

            if (tentative_g < neighbor.g) {
                neighbor.g = tentative_g;
                neighbor.h = heuristic(neighbor, goal);
                neighbor.parent = &current;
                open_set.push(neighbor);
            }
        }
    }

    return {};
}

void draw_path(vector<vector<int>>& grid, const vector<pair<int, int>>& path) {
    for (const auto& point : path) {
        grid[point.first][point.second] = -1; // Marking the path
    }
}

int main() {
    vector<vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    for (const auto& row : grid) {
        for (int cell : row) {
            cout << cell << " ";
        }
        cout << endl;
    }

    Node start_node{0, 0};
    Node goal_node{2, 3};

    vector<pair<int, int>> path = a_star(grid, start_node, goal_node);

    if (!path.empty()) {
        cout << "\nPath found:" << endl;
        draw_path(grid, path);

        for (const auto& row : grid) {
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
