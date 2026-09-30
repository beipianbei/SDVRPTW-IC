#ifndef DISTANCE_UTILS_H
#define DISTANCE_UTILS_H

#include <cmath>

inline double roundRoutingDistance2(double value)
{
	return std::round(value * 100.0) / 100.0;
}

inline double euclideanRoutingDistance2(double x1, double y1, double x2, double y2)
{
	const double dx = x1 - x2;
	const double dy = y1 - y2;
	return roundRoutingDistance2(std::sqrt(dx * dx + dy * dy));
}

#endif
