#ifndef LIDAR_LOC_UTILS_HPP
#define LIDAR_LOC_UTILS_HPP

#include <cmath>

inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

#endif // LIDAR_LOC_UTILS_HPP