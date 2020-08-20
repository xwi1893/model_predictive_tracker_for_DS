#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Point
{
public:
	double x;
	double y;
	double theta;
public:
	Point(double x, double y, double th = 0) : x(x), y(y), theta(th) {}
	Point(Vector2d &u, double th = 0): x(u(0)), y(u(1)), theta(th) {}

	Vector2d operator- (Point &p)
	{
		double x_ = x - p.x;
		double y_ = y - p.y;
		Vector2d u(x_, y_);
		return u;
	}

	Point operator+ (Vector2d &u)
	{
		double x_ = x + u(0);
		double y_ = y + u(1);
		return Point(x_, y_, theta);
	}


	double orientation(Point &p)
	{
		return atan2(y - p.y, x - p.x);
	}

	double distance(Point &p)
	{
		return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y));
	}

	bool transform2b(double &b, vector<Point> &bound)
	{
		auto fLine = [this](Point p) {return cos(theta)*(p.x - x) + sin(theta)*(p.y - y); };
		int index = 0;
		double fx1, fx2;

		if (bound.empty())
		{
			b = -5.0;
			return false;
		}
		else
		{
			fx1 = fLine(bound[0]);
			for (int i = 1; i < bound.size(); i++)
			{
				fx2 = fLine(bound[i]);
				if (fx1 * fx2 > 0) swap(fx1, fx2);
				else {
					double x0 = bound[i - 1].x;
					double y0 = bound[i - 1].y;
					double x1 = bound[i].x;
					double y1 = bound[i].y;
					Matrix2d Aeq;
					Aeq << cos(theta), sin(theta), y1 - y0, x0 - x1;
					Vector2d beq;
					beq << x * cos(theta) + y * sin(theta), x0*y1 - x1 * y0;
					Vector2d ps = Aeq.inverse()*beq;
					Point intersect(ps);
					int sgn = (sin(theta)*(intersect.x - x) + cos(theta)*(intersect.y - y) > 0) ? 1 : -1;
					b = this->distance(intersect)*sgn;
					return true;
				}
			}
			return false;
		}
	}

	friend ostream& operator<<(ostream &os, const Point &p)
	{
		os << "The x coordinate is " << p.x << ", the y coordinate is " << p.y << '.';
		return os;
	}
};