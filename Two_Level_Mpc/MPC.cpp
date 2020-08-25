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

double MPC::linear_interpolation(double x, vector<double> &xs, vector<double> &ys)
{
	if (xs.size() != ys.size()) throw "The lookup table is not the same size!";

	int i1 = 0;
	int i2 = xs.size() - 1;
	while (i2 - i1 > 1)
	{
		int i_tmp = (i1 + i2) / 2;
		if (xs[i_tmp] <= x) i1 = i_tmp;
		else i2 = i_tmp;
	}
	return (xs[i2] - x) / (xs[i2] - xs[i1])*ys[i1] + (x - xs[i1]) / (xs[i2] - xs[i1])*ys[i1];
}

void MPC::linear_interpolation(vector<double>& x, vector<double>& y, vector<double>& xs, vector<double>& ys)
{
	if (xs.size() != ys.size()) throw "The lookup table is not the same size!";

	for (auto iter = x.begin(); iter != x.end(); iter++) y.push_back(linear_interpolation(*iter, xs, ys));
}


