#include "MPC.h"

int nblkdiag(MatrixXd &M, MatrixXd &N, int n)
{
	if (N.rows() != M.rows()*n || N.cols() != M.cols()*n) return 0;
	for (int i = 0; i < n; i++)
	{
		N.block(i*M.rows(), i*M.cols(), M.rows(), M.cols()) = M.block(0, 0, M.rows(), M.cols());
	}
	return 1;
}

double MPC::linear_interpolation(double x, vector<double> &xs, vector<double> &ys, size_t start_index)
{
	auto intersect = [](double x0, double y0, double x1, double y1, double x) {return (x1 - x) / (x1 - x0)*y0 + (x - x0) / (x1 - x0)*y1; };
	if (xs.size() != ys.size()) throw "The lookup table is not the same size!";
	
	while (xs[start_index] <= x)
	{
		if ((xs[start_index + 1] > x) | (start_index == xs.size() - 1)) return intersect(xs[start_index], ys[start_index], xs[start_index + 1], ys[start_index + 1], x);
		else start_index++;
	}
	while (xs[start_index] > x)
	{
		if ((xs[start_index - 1] <= x) | (start_index == 1)) return intersect(xs[start_index - 1], ys[start_index - 1], xs[start_index], ys[start_index], x);
		else start_index--;
	}
}

void MPC::linear_interpolation(vector<double>& x, vector<double>& y, vector<double>& xs, vector<double>& ys, size_t start_index)
{
	auto intersect = [](double x0, double y0, double x1, double y1, double x) {return (x1 - x) / (x1 - x0)*y0 + (x - x0) / (x1 - x0)*y1; };
	if (xs.size() != ys.size()) throw "The lookup table is not the same size!";

	for (auto iter = x.begin(); iter != x.end(); iter++)
	{
		while (xs[start_index] <= *iter)
		{
			if ((xs[start_index + 1] > *iter) | (start_index == xs.size() - 1)) {
				double ans = intersect(xs[start_index], ys[start_index], xs[start_index + 1], ys[start_index + 1], *iter);
				y.push_back(ans);
				break;
			}
			else start_index++;
		}
		while (xs[start_index] > *iter)
		{
			if ((xs[start_index - 1] <= *iter) | (start_index == 1)) {
				double ans = intersect(xs[start_index - 1], ys[start_index - 1], xs[start_index], ys[start_index], *iter);
				y.push_back(ans);
				break;
			}
			else start_index--;
		}
	}
}


