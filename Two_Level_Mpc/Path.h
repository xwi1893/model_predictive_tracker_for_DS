#pragma once
#include "Point.h"

using namespace std;

class Path
{
public:
	Path() {}

	Path(vector<Point> &points, vector<double> &kappa, vector<double> &s): waypoints(points), kappa(kappa), s(s) {}
	Path(const Path &p): waypoints(p.waypoints), kappa(p.kappa), s(p.s) { isCopied = false; }
	~Path() {}

	Point linear_interpolation(double x, size_t &start_index)
	{
		auto intersect = [](double x0, double x1, double x) -> pair<double, double> {return make_pair((x1 - x) / (x1 - x0), (x - x0) / (x1 - x0)); };
		if (start_index < 1 || start_index >= s.size() - 1) start_index = 1;
		Point *p1, *p2;
		int i1, i2;
		if (s[start_index] <= x) {
			i1 = start_index;
			i2 = s.size() - 1;
		}
		else
		{
			i1 = 0;
			i2 = start_index;
		}

		while (i2 - i1 > 1)
		{
			int i_tmp = (i1 + i2) / 2;
			if (s[i_tmp] <= x) i1 = i_tmp;
			else i2 = i_tmp;
		}
		start_index = (size_t)i1;
		auto ratio_pair = intersect(s[i1], s[i2], x);
		p1 = &waypoints[i1];
		p2 = &waypoints[i2];
		
		double px = ratio_pair.first*p1->x + ratio_pair.second*p2->x;
		double py = ratio_pair.first*p1->y + ratio_pair.second*p2->y;
		double ptheta = ratio_pair.first*p1->theta + ratio_pair.second*p2->theta;
		return Point(px, py, ptheta);
	}

	void linear_interpolation(vector<double> &x, vector<Point> &pointPre)
	{
		auto intersect = [](double x0, double x1, double x) -> pair<double, double> {return make_pair((x1 - x) / (x1 - x0), (x - x0) / (x1 - x0)); };
		size_t start_index = s.size() / 2;
		linear_interpolation(x[0], start_index);

		for (auto iter = x.begin(); iter != x.end(); iter++)
		{
			while (s[start_index] <= *iter)
			{
				if ((s[start_index + 1] > *iter) || (start_index == s.size() - 2)) {
					auto ratio_pair = intersect(s[start_index], s[start_index + 1], *iter);
					Point &p1 = waypoints[start_index];
					Point &p2 = waypoints[start_index + 1];
					double px = ratio_pair.first*p1.x + ratio_pair.second*p2.x;
					double py = ratio_pair.first*p1.y + ratio_pair.second*p2.y;
					double ptheta = ratio_pair.first*p1.theta + ratio_pair.second*p2.theta;
					pointPre.emplace_back(px, py, ptheta);
					break;
				}
				else start_index++;
			}
		}
	}

	void linear_interpolation(vector<double> &x, vector<double> &kappaPre)
	{
		auto intersect = [](double x0, double x1, double x) -> pair<double, double> {return make_pair((x1 - x) / (x1 - x0), (x - x0) / (x1 - x0)); };
		size_t start_index = s.size() / 2;
		linear_interpolation(x[0], start_index);

		for (auto iter = x.begin(); iter != x.end(); iter++)
		{
			while (s[start_index] <= *iter)
			{
				if ((s[start_index + 1] > *iter) || (start_index == s.size() - 2)) {
					auto ratio_pair = intersect(s[start_index], s[start_index + 1], *iter);
					double pKappa = ratio_pair.first*kappa[start_index] + ratio_pair.second*kappa[start_index + 1];
					kappaPre.push_back(pKappa);
					break;
				}
				else start_index++;
			}
		}
	}

	void pop_first()
	{
		waypoints.erase(waypoints.begin());
		kappa.erase(kappa.begin());
		s.erase(s.begin());
	}

	void push_back(Point &p, double k, double s_)
	{
		waypoints.push_back(p);
		kappa.push_back(k);
		s.push_back(s_);
	}

	size_t size()
	{
		if ((waypoints.size() != kappa.size()) | (kappa.size() != s.size())) throw "The size of path does not match!";
		return waypoints.size();
	}

	void clear() noexcept
	{
		waypoints.clear();
		kappa.clear();
		s.clear();
	}

	Path &operator=(const Path &p)
	{
		waypoints = p.waypoints;
		kappa = p.kappa;
		s = p.s;
		isCopied = false;
		return *this;
	}

public:
	vector<Point> waypoints;
	vector<double> kappa;
	vector<double> s;

	pair<Point, double> beginState;

	mutex pathMutex;
	condition_variable replanCv;
	bool isNeedReplan = true;
	bool isCopied = false;
};