#pragma once
#include "Point.h"

using namespace std;

class Path
{
public:
	Path() {}

	Path(vector<Point> &points, vector<double> &kappa, vector<double> &s): waypoints(points), kappa(kappa), s(s) {}
	Path(const Path &path) {
		waypoints = path.waypoints;
		kappa = path.kappa;
		s = path.s;
	}
	~Path() {}

	Point linear_interpolation(double x, size_t &start_index)
	{
		auto intersect = [](double x0, double x1, double x) -> pair<double, double> {return make_pair((x1 - x) / (x1 - x0), (x - x0) / (x1 - x0)); };
		if (start_index < 1) start_index = 1;
		Point *p1, *p2;
		pair<double, double> ratio_pair;

		if (s[start_index] <= x)
		{
			while (start_index < s.size() - 2)
			{
				if (s[start_index + 1] > x) break;
				else start_index++;
			}
			ratio_pair = intersect(s[start_index], s[start_index + 1], x);
			p1 = &waypoints[start_index];
			p2 = &waypoints[start_index + 1];
		}
		else
		{
			while (start_index > 1)
			{
				if (s[start_index - 1] < x) break;
				else start_index--;
			}
			ratio_pair = intersect(s[start_index - 1], s[start_index], x);
			p1 = &waypoints[start_index - 1];
			p2 = &waypoints[start_index];
		}
		double px = ratio_pair.first*p1->x + ratio_pair.second*p2->x;
		double py = ratio_pair.first*p1->y + ratio_pair.second*p2->y;
		double ptheta = ratio_pair.first*p1->theta + ratio_pair.second*p2->theta;
		return Point(px, py, ptheta);
	}

	void linear_interpolation(vector<double> &x, vector<Point> &pointPre, size_t start_index = 1)
	{
		auto intersect = [](double x0, double x1, double x) -> pair<double, double> {return make_pair((x1 - x) / (x1 - x0), (x - x0) / (x1 - x0)); };
		vector<double> &sSample = s;
		if (start_index < 1) start_index = 1;

		Point *p1, *p2;
		pair<double, double> ratio_pair;
		for (auto iter = x.begin(); iter != x.end(); iter++)
		{
			if (sSample[start_index] <= *iter)
			{
				while (start_index < sSample.size() - 2)
				{
					if (sSample[start_index + 1] > *iter) break;
					else start_index++;
				}
				ratio_pair = intersect(sSample[start_index], sSample[start_index + 1], *iter);
				p1 = &waypoints[start_index];
				p2 = &waypoints[start_index + 1];
			}
			else
			{
				while (start_index > 1)
				{
					if (sSample[start_index - 1] < *iter) break;
					else start_index--;
				}
				ratio_pair = intersect(sSample[start_index - 1], sSample[start_index], *iter);
				p1 = &waypoints[start_index - 1];
				p2 = &waypoints[start_index];
			}

			double px = ratio_pair.first*p1->x + ratio_pair.second*p2->x;
			double py = ratio_pair.first*p1->y + ratio_pair.second*p2->y;
			double ptheta = ratio_pair.first*p1->theta + ratio_pair.second*p2->theta;
			pointPre.emplace_back(px, py, ptheta);
		}
	}

	void linear_interpolation(vector<double> &x, vector<double> &kappaPre, size_t start_index = 0)
	{
		auto intersect = [](double x0, double x1, double x) -> pair<double, double> {return make_pair((x1 - x) / (x1 - x0), (x - x0) / (x1 - x0)); };
		vector<double> &sSample = s;
		for (auto iter = x.begin(); iter != x.end(); iter++)
		{
			while (sSample[start_index] <= *iter)
			{
				if ((sSample[start_index + 1] > *iter) | (start_index == sSample.size() - 1)) {
					auto ratio_pair = intersect(sSample[start_index], sSample[start_index + 1], *iter);
					double pKappa = ratio_pair.first*kappa[start_index] + ratio_pair.second*kappa[start_index + 1];
					kappaPre.push_back(pKappa);
					break;
				}
				else start_index++;
			}
			while (sSample[start_index] > *iter)
			{
				if ((sSample[start_index - 1] <= *iter) | (start_index == 1)) {
					auto ratio_pair = intersect(sSample[start_index - 1], sSample[start_index], *iter);
					double pKappa = ratio_pair.first*kappa[start_index - 1] + ratio_pair.second*kappa[start_index];
					kappaPre.push_back(pKappa);

					break;
				}

				else start_index--;
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

	Path operator+(Path &path)
	{
		Path p(*this);
		p.waypoints.insert(p.waypoints.end(), path.waypoints.begin(), path.waypoints.end());
		p.kappa.insert(p.kappa.end(), path.kappa.begin(), path.kappa.end());
		p.s.insert(p.s.end(), path.s.begin(), path.s.end());
		return p;
	}

	Path &operator=(const Path &path)
	{
		if (this == &path) return *this;
		waypoints = path.waypoints;
		kappa = path.kappa;
		s = path.s;
		return *this;
	}
public:
	vector<Point> waypoints;
	vector<double> kappa;
	vector<double> s;
};
