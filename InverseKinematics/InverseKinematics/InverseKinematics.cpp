#include <iostream>
#include <math.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// dh parameter structure
struct DH_params
{
	double a;
	double alpha;
	double d;
};

// function to calculate the homogeneous transformation matrix for a single joint
Matrix4d HomogeneousTransformation(double a, double alpha, double d, double theta)
{
	Matrix4d T;
	T << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta),
		sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
		0, sin(alpha), cos(alpha), d,
		0, 0, 0, 1;
	return T;
}
VectorXd InverseKinematics(DH_params dh_params[6], double x, double y, double z, double roll, double pitch, double yaw)
{
	// end effector's rotation matrix
	Matrix3d R;
	R << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
		sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll),
		-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll);

	// initialize joint angles with zero
	VectorXd joint_angles(6);
	joint_angles.setZero();

	// the position of the end effector with respect to the base frame
	Vector3d p_e = R.transpose() * Vector3d(x, y, z);
	p_e = -p_e;

	// joint angles using closed-form solution method
	joint_angles(0) = atan2(p_e(1), p_e(0)) - atan2(dh_params[0].d, sqrt(p_e(0)*p_e(0) + p_e(1)*p_e(1) - dh_params[0].d*dh_params[0].d));
	joint_angles(1) = atan2(sqrt(p_e(0)*p_e(0) + p_e(1)*p_e(1) - dh_params[0].d*dh_params[0].d), -dh_params[0].d) + atan2(p_e(2) - dh_params[0].a, sqrt(p_e(0)*p_e(0) + p_e(1)*p_e(1) - dh_params[0].d*dh_params[0].d));
	joint_angles(2) = atan2(dh_params[2].d, dh_params[1].a) - atan2(sqrt(p_e(0)*p_e(0) + p_e(1)*p_e(1) - dh_params[0].d*dh_params[0].d), p_e(2) - dh_params[0].a);

	// rotation matrix of the end effector with respect to the base frame
	Matrix3d R_b_e;
	R_b_e.col(0) = R.col(0);
	R_b_e.col(1) = R.col(1);
	R_b_e.col(2) = R.col(2);

	// joint angles for the remaining joints
	for (int i = 3; i < 6; i++)
	{
		Matrix3d R_b_j = HomogeneousTransformation(dh_params[i - 1].a, dh_params[i - 1].d, dh_params[i - 1].alpha, joint_angles(i - 1)).block<3, 3>(0, 0);
		Matrix3d R_j_e = R_b_j.transpose() * R_b_e;
		joint_angles(i) = atan2(sqrt(R_j_e(0, 2)*R_j_e(0, 2) + R_j_e(2, 2)*R_j_e(2, 2)), R_j_e(1, 2));
		if (R_j_e(2, 2) < 0) joint_angles(i) = -joint_angles(i);
		if (R_j_e(0, 2) < 0) joint_angles(i) = -joint_angles(i);
		joint_angles(i) += joint_angles(i - 1);
	}

	return joint_angles;
}
int main()
{
	// dh parameters init
	DH_params dh_params[6] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

	// ee position and orientation
	double x, y, z, roll, pitch, yaw;

	// input dh parameters and end effector's position and orientation
	cout << "enter  parameters (a, alpha, d) for each joints: " << endl;
	for (int i = 0; i < 6; i++)
	{
		cout << "joint " << i + 1 << ": ";
		cin >> dh_params[i].a >> dh_params[i].alpha >> dh_params[i].d;
	}
	cout << "enter end effector's position";
	cin >> x >> y >> z >> roll >> pitch >> yaw;

	VectorXd joint_angles = InverseKinematics(dh_params, x, y, z, roll, pitch, yaw);
	cout << "joint angles are:\n";
	cout << joint_angles;
}

