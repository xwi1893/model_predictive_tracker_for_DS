#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Waypoint {
public: 
	double x, y, theta, kappa, s, offset;

public:
	Waypoint() {}
	Waypoint(double s, double offset) : x(0), y(0), theta(0), kappa(0), s(s), offset(offset) {}
	Waypoint(double x, double y, double s, double offset, double theta = 0, double kappa = 0) : x(x), y(y), theta(theta), kappa(kappa), s(s), offset(offset) {}
	Waypoint(const Waypoint &p) : x(p.x), y(p.y), theta(p.theta), kappa(p.kappa), s(p.s), offset(p.offset) {}
	
	Vector2d operator- (const Waypoint &p)
	{
		Vector2d u(x - p.x, y - p.y);
		return u;
	}

	Waypoint operator+ (const Waypoint &p)
	{
		return Waypoint(x + p.x, y + p.y, s + p.s, offset + p.offset, theta + p.theta, kappa + p.kappa);
	}

	Waypoint operator* (double weight)
	{
		return Waypoint(weight*x, weight*y, weight*s, weight*offset, weight*theta, weight*kappa);
	}

	Waypoint &operator=(const Waypoint& p)
	{
		x = p.x; y = p.y; s = p.s; offset = p.offset; theta = p.theta; kappa = p.kappa;
		return *this;
	}

	double orientation(const Waypoint &p)
	{
		return atan2(y - p.y, x - p.x);
	}

	double distance(double px, double py) {
		return sqrt((x - px)*(x - px) + (y - py)*(y - py));
	}

	friend ostream& operator<<(ostream &os, const Waypoint &p)
	{
		os << "The x coordinate is " << p.x << ", the y coordinate is " << p.y << '.';
		return os;
	}

};