#include <iostream>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;
double PI = 3.14159265;

// converting roll-pitch-yaw angles to rotation matrix
Matrix3d rpy2rot(double roll, double pitch, double yaw)
{
	Matrix3d R_x, R_y, R_z;

	R_x << 1, 0, 0,
		0, cos(roll), -sin(roll),
		0, sin(roll), cos(roll);

	R_y << cos(pitch), 0, sin(pitch),
		0, 1, 0,
		-sin(pitch), 0, cos(pitch);

	R_z << cos(yaw), -sin(yaw), 0,
		sin(yaw), cos(yaw), 0,
		0, 0, 1;

	Matrix3d R = R_z * R_y * R_x;

	return R;
}

// calculating the inverse kinematics
VectorXd inverse_kinematics(double a[], double alpha[], double d[], double x, double y, double z, double roll, double pitch, double yaw)
{
	Matrix4d T_0_6;
	T_0_6.setIdentity();

	T_0_6.block<3, 3>(0, 0) = rpy2rot(roll, pitch, yaw);
	T_0_6(0, 3) = x;
	T_0_6(1, 3) = y;
	T_0_6(2, 3) = z;

	VectorXd theta(6);

	// first joint
	theta[0] = atan2(T_0_6(1, 3), T_0_6(0, 3));

	// second joint
	double c2 = (pow(T_0_6(0, 3), 2) + pow(T_0_6(1, 3), 2) - pow(a[0], 2) - pow(a[1], 2)) / (2 * a[0] * a[1]);
	double s2 = sqrt(1 - pow(c2, 2));
	theta[1] = atan2(s2, c2);

	// third joint
	double c3 = (pow(T_0_6(0, 3), 2) + pow(T_0_6(1, 3), 2) + pow(a[0], 2) + pow(a[1], 2) - pow(d[2], 2)) / (2 * a[0] * a[1]);
	double s3 = (T_0_6(2, 3) - d[0]) / d[2];
	theta[2] = atan2(s3, c3);

	// fourth joint
	double c4 = (T_0_6(0, 0) * T_0_6(0, 0) + T_0_6(1, 0) * T_0_6(1, 0) + T_0_6(2, 0) * T_0_6(2, 0) - pow(a[3], 2) - pow(d[3], 2)) / (2 * a[3] * d[3]);
	double s4 = sqrt(1 - pow(c4, 2));
	theta[3] = atan2(s4, c4);

	// fifth joint
	double c5 = T_0_6(0, 0);
	double s5 = T_0_6(2, 0);
	theta[4] = atan2(s5, c5);

	// sixth joint
	double c6 = T_0_6(0, 1);
	double s6 = T_0_6(1, 1);
	theta[5] = atan2(s6, c6);

	return theta;
}

int main()
{
	double a[6];
	std::cout << "input DH a parameters for every joint ";
		for (int i = 0; i < 6; i++) {
		std::cin >> a[i];
	}

	double alpha[6];
	std::cout << "input DH alpha parameters for every joint ";
	for (int i = 0; i < 6; i++) {
		std::cin >> alpha[i];
	}

	double d[6];
	std::cout << "input DH d parameters for every joint ";
	for (int i = 0; i < 6; i++) {
		std::cin >> d[i];
	}

	double x, y, z, roll, pitch, yaw;
	std::cout << "input x y z roll pitch yaw ";
	std::cin >> x >> y >> z >> roll >> pitch >> yaw;
	

	VectorXd theta = inverse_kinematics(a, alpha, d, x, y, z, roll, pitch, yaw);

	std::cout << "joint angles are: " << std::endl << theta << std::endl;

	return 0;
}