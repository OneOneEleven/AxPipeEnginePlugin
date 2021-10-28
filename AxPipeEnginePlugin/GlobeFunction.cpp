#include "GlobeFunction.h"
#include <algorithm>

double GlobalFun::getDoubleMAXIMUM()
{
	return (std::numeric_limits<double>::max)();
}

double GlobalFun::computeEulerDist(Eigen::Vector3f & p1, Eigen::Vector3f & p2)
{
	double dist2 = (p1 - p2).norm();
	if (dist2 < 1e-8 || dist2 > 1e8)
	{
		return 0;
	}
	return sqrt(dist2);
}

double GlobalFun::computeEulerDistSquare(Eigen::Vector3f & p1, Eigen::Vector3f & p2)
{
	return (p1 - p2).norm();
}

double GlobalFun::computeProjDist(Eigen::Vector3f & p1, Eigen::Vector3f & p2, Eigen::Vector3f & normal_of_p1)
{
	normal_of_p1.normalize();
	return (p2 - p1).dot(normal_of_p1);
}

bool GlobalFun::isTwoPoint3fTheSame(Eigen::Vector3f& v0, Eigen::Vector3f& v1)
{
	if (abs(v0[0] - v1[0]) < 1e-7 &&
		abs(v0[1] - v1[1]) < 1e-7 &&
		abs(v0[2] - v1[2]) < 1e-7)
	{
		return true;
	}

	return false;

}

bool GlobalFun::isTwoPoint3fOpposite(Eigen::Vector3f& v0, Eigen::Vector3f& v1)
{
	if (abs(-v0[0] - v1[0]) < 1e-7 &&
		abs(-v0[1] - v1[1]) < 1e-7 &&
		abs(-v0[2] - v1[2]) < 1e-7)
	{
		return true;
	}

	return false;
}

double GlobalFun::computeRealAngleOfTwoVertor(Eigen::Vector3f& v0, Eigen::Vector3f& v1)
{
	v0.normalize();
	v1.normalize();


	if (isTwoPoint3fTheSame(v0, v1))
	{
		return 0;
	}

	if (isTwoPoint3fOpposite(v0, v1))
	{
		return 180;
	}

	double angle_cos = v0.dot(v1);
	if (angle_cos > 1)
	{
		angle_cos = 0.99;
	}
	if (angle_cos < -1)
	{
		angle_cos = -0.99;
	}
	if (angle_cos > 0 && angle_cos < 1e-8)
	{
		return 90;
	}

	double angle = acos(angle_cos) * 180. / 3.1415926;

	if (angle < 0 || angle > 180)
	{
		// "compute angle wrong!!" << endl;
		//system("Pause");
		return -1;
	}
	return angle;
}