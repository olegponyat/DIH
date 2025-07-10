#ifndef PATH_H
#define PATH_H

#include <vector>
#include <utility>
#include "position.h"

// Type aliases for convenience
using pii = std::pair<int, int>;
using vii = std::vector<pii>;

namespace path {
    /**
     * @brief Finds a path from a start point to a goal or bonus point using BFS.
     * 
     * @param start The starting coordinates (row, col).
     * @param solved A reference to a boolean that will be set to true if the goal (2) is reached.
     * @return A vector of pairs representing the path from start to the target. Returns an empty vector if no path is found.
     */
    vii BFS(pii start, bool &solved);
}

void toAbsoluteCoordinates(PathVector &path);
void interpolatePath(PathVector &path, const Position &start, const Position &end);
void generatePath(PathVector &path, std::vector<PathSegment> &result);

#endif // PATH_H
