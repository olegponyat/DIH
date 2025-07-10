#include "path.h"

#include <algorithm>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <vector>
#include <set>
#include <stdio.h>

#include "position.h"

static const int SPEED = 50;

namespace path {

const int ROWS = 7;
const int COLS = 9;

/*
s = start, l = grid line
0 = grid, 1 = barrier, 2 = goal,
3 = intersection (impossible to get to, intersection between grid lines)
4 = bonus points

Directions:
N,E,S,W

N = towards y = 0
W = towards x = 0
*/
int grid[ROWS][COLS] = {
    {0, 0, 0, 0, 0, 0, 2, 0, 0},
    {0, 3, 0, 3, 1, 3, 1, 0, 0}, // <- l
    {0, 0, 4, 1, 0, 0, 0, 0, 0},
    {0, 3, 1, 3, 0, 3, 0, 0, 0}, // <- l
    {0, 0, 0, 0, 0, 1, 4, 0, 0},
    {1, 3, 1, 3, 0, 3, 1, 0, 0}, // <- l
    {0, 0, 0, 0, 0, 0, 0, 0, 0}
};

bool vis[ROWS][COLS];

// Movement vectors for grid cells (skips grid lines)
int dy[4] = {-2, 2, 0, 0}; // N, S
int dx[4] = {0, 0, 2, -2}; // E, W

// Barrier check vectors (on grid lines)
int gy[4] = {-1, 1, 0, 0}; // N, S
int gx[4] = {0, 0, 1, -1}; // E, W

void clearVis() {
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            vis[i][j] = false;
        }
    }
}

vii BFS(pii start, bool &solved) {
    clearVis();
    std::queue<pii> q;
    std::map<pii, pii> parent;
    
    solved = false;
    vis[start.first][start.second] = true;
    q.push(start);
    
    while (!q.empty()) {
        pii current = q.front();
        q.pop();
        int y = current.first;
        int x = current.second;
        
        if (grid[y][x] == 4 || grid[y][x] == 2) {
            if (grid[y][x] == 2) {
                solved = true;
            }
            vii path;
            while (current != start) {
                path.push_back(current);
                current = parent[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        for (int i = 0; i < 4; i++) {
            int ny = y + dy[i];
            int nx = x + dx[i];
            int ngy = y + gy[i];
            int ngx = x + gx[i];
            
            if (ny >= 0 && ny < ROWS && nx >= 0 && nx < COLS) {
                if (grid[ngy][ngx] != 0) continue; // Check for barrier on grid line
                if (vis[ny][nx]) continue;
                
                pii neighbor = std::make_pair(ny, nx);
                vis[ny][nx] = true;
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }
    
    return vii(); // Return empty path if no target found
}

} // namespace path

void toAbsoluteCoordinates(PathVector &path) {
  for (int i = 0; i < path.size(); i++) {
    Position pos = path[i];

    pos.x = 50 * pos.x + 25;
    pos.y = 50 * pos.y + 25;

    path[i] = pos;
  }

  printf("size: %d\n", path.size());
}

void interpolatePath(PathVector &path, const Position &start,
                     const Position &end) {
  double d = start.distance(end);

  for (double n = 1; n < d; n++) {
    path.push_back(Position{/* .x = */ start.x + n / d * (end.x - start.x),
                            /* .y = */ start.y + n / d * (end.y - start.y),
                            /* .theta = */ SPEED});
  }
}

void generatePath(PathVector &path, std::vector<PathSegment> &result) {
  std::optional<Position> prev;
  PathVector currentPath;

  if (path.size() < 2) {
    if (!path.empty()) {
        result.push_back({0, path});
    }
    return;
  }

  for (int i = 0; i < path.size() - 1; i++) {
    Position start = path.at(i);
    Position end = path.at(i + 1);

    // interpolate between current and next
    interpolatePath(currentPath, start, end);

    // if last part, set speed 0
    currentPath.push_back(
        Position{end.x, end.y, (i + 1 == path.size() - 1) ? 0 : SPEED});

    // check if turning
    if (i < path.size() - 2) {
      Position next = path.at(i + 2);
      // if not straight line
      if (start.angle(end) != end.angle(next)) {
        currentPath.pop_back();
        currentPath.push_back({end.x, end.y, 0});

        result.push_back({
            0, // field not used atm
            currentPath,
        });
        result.push_back({end.angle(next), {}});

        currentPath.clear();
      }
    }

    prev = start;
  }

  // push in last segment if it wasn't part of a turn
  if (!currentPath.empty()) {
    result.push_back({0, currentPath});
  }
}
