#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class Taylor {
public:
	Taylor() {}

	~Taylor() {}

	 static void Cxy(double x, double y, double v, double a, double theta, vector<MatrixXd>& coef) {
		 coef.push_back((MatrixXd(2, 1) << x, y).finished());
		 coef.push_back((MatrixXd(2, 1) << v * cos(theta), v*sin(theta)).finished());
		 coef.push_back((MatrixXd(2, 1) << a * cos(theta) / 2, a*sin(theta) / 2).finished());
		 coef.push_back((MatrixXd::Zero(2, 1)));
	 }

	 static void CR(double alpha, double dalpha, vector<MatrixXd>& coef) {
		 coef.push_back((MatrixXd(2, 2) << cos(alpha), -sin(alpha), sin(alpha), cos(alpha)).finished());
		 coef.push_back((MatrixXd(2, 2) << -dalpha * sin(alpha), -dalpha * cos(alpha),
											dalpha * cos(alpha), -dalpha * sin(alpha)).finished());
		 coef.push_back((MatrixXd(2, 2) << -dalpha * dalpha*cos(alpha) / 2.0, dalpha*dalpha*sin(alpha) / 2.0,
										   -dalpha * dalpha*sin(alpha) / 2.0, -dalpha * dalpha*cos(alpha) / 2.0).finished());
		 coef.push_back((MatrixXd(2, 2) << pow(dalpha, 3)*sin(alpha) / 6.0, pow(dalpha, 3)*cos(alpha) / 6.0,
										  -pow(dalpha, 3)*cos(alpha) / 6.0, pow(dalpha, 3)*sin(alpha) / 6.0).finished());
	 }

	 static void ClambdaR(double alpha, double dalpha, double dsk, double dsv, double sk, double sv, vector<MatrixXd>& coef) {
		 coef.push_back((MatrixXd(2, 2) << sk * cos(alpha) / sv, -sk * sin(alpha) / sv, sk * sin(alpha) / sv, sk*cos(alpha) / sv).finished());
		 coef.push_back((MatrixXd(2, 2) << (dsk*cos(alpha) - dalpha * sk*sin(alpha)) / sv - dsv * sk / (sv*sv)*cos(alpha),
			 -(dsk*sin(alpha) + dalpha * sk*cos(alpha)) / sv + dsv * sk / (sv*sv)*sin(alpha),
			 (dsk*sin(alpha) + dalpha * sk*cos(alpha)) / sv - dsv * sk / (sv*sv)*sin(alpha),
			 (dsk*cos(alpha) - dalpha * sk*sin(alpha)) / sv - dsv * sk / (sv*sv)*cos(alpha)).finished());
		 coef.push_back((MatrixXd(2, 2) << -((dalpha*dalpha*sk*cos(alpha)) / 2.0 + dalpha * dsk*sin(alpha)) / sv - dsv / (sv*sv)*(dsk*cos(alpha) - dalpha * sk*sin(alpha)) + dsv * dsv*sk / (sv*sv*sv)*cos(alpha),
			 ((dalpha*dalpha*sk*sin(alpha)) / 2.0 - dalpha * dsk*cos(alpha)) / sv + dsv / (sv*sv)*(dsk*sin(alpha) + dalpha * sk*cos(alpha)) - dsv * dsv*sk / (sv*sv*sv)*sin(alpha),
			 -((dalpha*dalpha*sk*sin(alpha)) / 2.0 - dalpha * dsk*cos(alpha)) / sv - dsv / (sv*sv)*(dsk*sin(alpha) + dalpha * sk*cos(alpha)) + dsv * dsv*sk / (sv*sv*sv)*sin(alpha),
			 -((dalpha*dalpha*sk*cos(alpha)) / 2.0 + dalpha * dsk*sin(alpha)) / sv - dsv / (sv*sv)*(dsk*cos(alpha) - dalpha * sk*sin(alpha)) + dsv * dsv*sk / (sv*sv*sv)*cos(alpha)).finished());
		 coef.push_back((MatrixXd(2, 2) << -((pow(dalpha, 2)*dsk*cos(alpha)) / 2.0 - (pow(dalpha, 3)*sk*sin(alpha)) / 6.0) / sv + dsv / (sv*sv)*((dalpha*dalpha*sk*cos(alpha)) / 2.0 + dalpha * dsk*sin(alpha)) + dsv * dsv / pow(sv, 3)*(dsk*cos(alpha) - dalpha * sk*sin(alpha)) - pow(dsv, 3)*sk / pow(sv, 4)*cos(alpha),
			 ((pow(dalpha, 3)*sk*cos(alpha)) / 6.0 + (pow(dalpha, 2)*dsk*sin(alpha)) / 2.0) / sv - dsv / (sv*sv)*((dalpha*dalpha*sk*sin(alpha)) / 2.0 - dalpha * dsk*cos(alpha)) - dsv * dsv / pow(sv, 3)*(dsk*sin(alpha) + dalpha * sk*cos(alpha)) + pow(dsv, 3)*sk / pow(sv, 4)*sin(alpha),
			 -((pow(dalpha, 3)*sk*cos(alpha)) / 6.0 + (pow(dalpha, 2)*dsk*sin(alpha)) / 2.0) / sv + dsv / (sv*sv)*((dalpha*dalpha*sk*sin(alpha)) / 2.0 - dalpha * dsk*cos(alpha)) + dsv * dsv / pow(sv, 3)*(dsk*sin(alpha) + dalpha * sk*cos(alpha)) - pow(dsv, 3)*sk / pow(sv, 4)*sin(alpha),
			 -((pow(dalpha, 2)*dsk*cos(alpha)) / 2.0 - (pow(dalpha, 3)*sk*sin(alpha)) / 6.0) / sv + dsv / (sv*sv)*((dalpha*dalpha*sk*cos(alpha)) / 2.0 + dalpha * dsk*sin(alpha)) + dsv * dsv / pow(sv, 3)*(dsk*cos(alpha) - dalpha * sk*sin(alpha)) - pow(dsv, 3)*sk / pow(sv, 4)*cos(alpha)).finished());
	 }
};