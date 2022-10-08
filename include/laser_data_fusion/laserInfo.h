#pragma once
#include <common/MathConstExpr.h>

constexpr double LOOP_RATE = 20;

constexpr int POINTS_PER_ANGLE = 4;
constexpr int POINTS_PER_LOOP = ANGLES_OF_LOOP * POINTS_PER_ANGLE;

constexpr double LASER_OFFSET[][3] = {{0.765, 0.365, 5 * PI / 4}, {-0.075, -0.365, PI / 4}};
constexpr double DIST_RANGE[] = {0.0, 12.0};
constexpr double ANGLE_RANGE[][2] = {{-PI / 4, PI / 4}, {-0.3 * PI, PI / 4}};
