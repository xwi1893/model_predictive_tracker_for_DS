#pragma once
#include "wayPoint.h"
#include <fstream>
#include <mutex>
#include <condition_variable>

using namespace std;


class Path
{
public:
	Path() {}

	Path(vector<Waypoint> &points, vector<double> &dkappads, vector<double> &s) : waypoints(points), dkappads(dkappads), s(s) {}
	Path(const Path &p) : waypoints(p.waypoints), dkappads(p.dkappads), s(p.s) {}
	~Path() {}

	int loadFile(const string &fileName) {
		clear();
		ifstream file;
		file.open(fileName, ios::in);
		if (!file.good()) return 0;

		while (file)
		{
			string str;
			if (!getline(file, str)) break;
			istringstream ss(str);
			vector<double> data;
			while (ss)
			{
				string str_data;
				if (!getline(ss, str_data, ',')) break;
				data.push_back(stod(str_data));
			}
			waypoints.emplace_back(data[0], data[1], data[2], 0, data[3], data[4]);
			dkappads.push_back(data[5]);
			s.push_back(data[2]);
		}
		file.close();
		return 1;
	}

	Waypoint cart2frenet(double x, double y, const size_t start, const size_t end) {
		// find closest point
		vector<size_t>	samples = { start, 0, 0, 0, end };
		size_t index_min;
		double d_min;
		while (samples[4]-samples[0]>=4)
		{
			auto s = samples[0], e = samples[4];
			samples[1] = s + (e - s) / 4;
			samples[2] = s + (e - s) / 2;
			samples[3] = s + 3 * (e - s) / 4;
			index_min = 1;
			d_min = waypoints[samples[1]].distance(x, y);

			for (int i = 2; i <= 3; i++) {
				double d_i = waypoints[samples[i]].distance(x, y);
				if (d_i < d_min) {
					d_min = d_i;
					index_min = i;
				}
			}
			samples[0] = samples[index_min - 1];
			samples[4] = samples[index_min + 1];
		}

		d_min = 1e6;
		for (size_t i = samples[0]; i <= samples[4]; i++) {
			double d_i = waypoints[i].distance(x, y);
			if (d_i < d_min) {
				d_min = d_i;
				index_min = i;
			}
		}

		Vector2d u, v; int sgn = 1;
		if (index_min < s.size() - 1) {
			u = waypoints[index_min+1] - waypoints[index_min];
			v = Vector2d(x-waypoints[index_min].x, y-waypoints[index_min].y);
			sgn = atan2(y - waypoints[index_min].y, x - waypoints[index_min].x) >= waypoints[index_min + 1].orientation(waypoints[index_min]) ? 1 : -1;
		}
		else {
			u = waypoints[index_min] - waypoints[index_min - 1];
			v = Vector2d(x - waypoints[index_min].x, y - waypoints[index_min].y);
			sgn = atan2(y - waypoints[index_min].y, x - waypoints[index_min].x) >= waypoints[index_min].orientation(waypoints[index_min - 1]) ? 1 : -1;
		}
		double angle = acos(u.dot(v) / (u.norm()*v.norm()))*sgn;
		double sp = s[index_min] + v.norm()*cos(angle);

		Waypoint p;
		interp<double, Waypoint>(s, waypoints, sp, p);
		p.offset = v.norm()*sin(angle);
		p.x = x; p.y = y;
		return p;
	}

	void pop_first()
	{
		waypoints.erase(waypoints.begin());
		dkappads.erase(dkappads.begin());
		s.erase(s.begin());
	}

	void push_back(Waypoint &p, double c, double s_)
	{
		waypoints.push_back(p);
		dkappads.push_back(c);
		s.push_back(s_);
	}

	size_t size()
	{
		if (waypoints.size() != s.size() || waypoints.size() != dkappads.size()) throw "The size of path does not match!";
		return waypoints.size();
	}

	void clear() noexcept
	{
		waypoints.clear();
		dkappads.clear();
		s.clear();
	}

	Path &operator=(const Path &p)
	{
		waypoints = p.waypoints;
		dkappads = p.dkappads;
		s = p.s;
//		isCopied = false;
		return *this;
	}

public:
	vector<Waypoint> waypoints;
	vector<double> dkappads;
	vector<double> s;

	mutex pathMutex;
	condition_variable replanCv;
	bool isNeedReplan = true;
//	bool isCopied = false;
};