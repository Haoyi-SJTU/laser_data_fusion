#pragma once
#include <common/MathConstExpr.h>

constexpr double LOOP_RATE = 20;

constexpr int POINTS_PER_ANGLE = 4;
constexpr int POINTS_PER_LOOP = ANGLES_OF_LOOP * POINTS_PER_ANGLE;

constexpr double LASER_OFFSET[][2] = {{0.8, 0.40}, {-0.10, -0.40}};
;
constexpr double DIST_RANGE[] = {0.0, 12.0};
