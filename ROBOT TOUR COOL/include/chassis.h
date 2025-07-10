#pragma once

#include "Motor.h"
#include "position.h"
#include <vector>

struct LRT {
  double left, right, theta;
};

namespace chassis {

extern MotorController driveLeftController;
extern MotorController driveRightController;
extern LRT *velocity;

/**
 * Handles one odometry tick
 */
void doOdometryTick();

/**
 * Runs odometry in a loop
 */
void odometryTask();

void initializeOdometry();

Position getPosition(bool degrees = false, bool standardPos = false);

void move(int left, int right);
void moveVelocity(int left, int right);

void follow(std::vector<Position> &pathPoints, float lookahead, int endTime,
            float remainingDistance);

void turnTo(float angle);

void setPose(const Position &newState, bool setTheta = false);

} // namespace chassis
