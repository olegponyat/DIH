#ifndef POSITION_H
#define POSITION_H

#include <vector>
#include <cmath>

struct Position {
    double x = 0, y = 0, theta = 0;

    bool equals(const Position &other) const {
        return x == other.x && y == other.y;
    }

    double distance(const Position &other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }

    double angle(const Position &other) const {
        return std::atan2(other.y - y, other.x - x) * 180.0 / M_PI;
    }
};

using PathVector = std::vector<Position>;

struct PathSegment {
    double angle;
    PathVector path;
};

#endif // POSITION_H
