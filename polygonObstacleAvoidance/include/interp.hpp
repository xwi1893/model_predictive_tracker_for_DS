#ifndef INTERP_HPP
#define INTERP_HPP

#include <cassert>
#include <vector>
#include <limits>

using namespace std;

template<typename Tx, typename Ty>
static void interp(vector<Tx>& xd, vector<Ty>& yd, Tx& x, Ty& y)
{
	size_t n = xd.size();
	assert(yd.size() == n);

	Tx weight; size_t mid;
	size_t lo = 0; size_t hi = n - 1;
	if (x < xd[0])
	{
		weight = 1;
	}
	else if (x > xd[n - 1]) {
		weight = 0;
	}
	else {
		while (lo < hi - 1)
		{
			mid = (hi + lo) / 2;
			if (x < xd[mid]) hi = mid;
			else if (x >= xd[mid]) lo = mid;
		}
		weight = (xd[hi] - x) / (xd[hi] - xd[lo]);
	}

	y = yd[lo] * weight + yd[hi] * (1 - weight);

}

template<typename Tx, typename Ty>
static void interp(vector<Tx>& xd, vector<Ty>& yd, vector<Tx>& xi, vector<Ty>& yi)
{
	size_t n = xd.size();
	assert(yd.size() == n);

	for (size_t i = 0; i < xi.size(); i++)
	{
		size_t mid;
		Tx weight;
		Tx x = xi[i];
		size_t lo = 0; size_t hi = n - 1;
		if (x < xd[0])
		{
			weight = 1;
		}
		else if (x > xd[n - 1]) {
			weight = 0;
		}
		else {
			while (lo < hi - 1)
			{
				mid = (hi + lo) / 2;
				if (x < xd[mid]) hi = mid;
				else if (x >= xd[mid]) lo = mid;
			}
			weight = (xd[hi] - x) / (xd[hi] - xd[lo]);
		}

		Ty y = yd[lo] * weight + yd[hi] * (1 - weight);

		if (yi.size() < i + 1) yi.push_back(y);
		else yi[i] = y;
	}
}

#endif

