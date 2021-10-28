#pragma once
#ifndef GLOBALFUN_HEADER
#define  GLOBALFUN_HEADER

#include <pcl/point_cloud.h>
namespace GlobalFun
{
	double computeEulerDist(Eigen::Vector3f & p1, Eigen::Vector3f & p2);
	double computeEulerDistSquare(Eigen::Vector3f & p1, Eigen::Vector3f & p2);
	double computeProjDist(Eigen::Vector3f & p1, Eigen::Vector3f & p2, Eigen::Vector3f & normal_of_p1);
	double computeProjDistSquare(Eigen::Vector3f & p1, Eigen::Vector3f & p2, Eigen::Vector3f & normal_of_p1);
	double computePerpendicularDistSquare(Eigen::Vector3f & p1, Eigen::Vector3f & p2, Eigen::Vector3f & normal_of_p1);
	double computePerpendicularDist(Eigen::Vector3f & p1, Eigen::Vector3f & p2, Eigen::Vector3f & normal_of_p1);
	double computeProjPlusPerpenDist(Eigen::Vector3f & p1, Eigen::Vector3f & p2, Eigen::Vector3f & normal_of_p1);
	double computeRealAngleOfTwoVertor(Eigen::Vector3f& v0, Eigen::Vector3f& v1);
	bool isTwoPoint3fOpposite(Eigen::Vector3f& v0, Eigen::Vector3f& v1);
	bool isTwoPoint3fTheSame(Eigen::Vector3f& v0, Eigen::Vector3f& v1);
	double getDoubleMAXIMUM();
};
#endif

