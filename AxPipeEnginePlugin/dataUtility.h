//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qOpenNDTSACPlugin           #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                             COPYRIGHT: Fan Yang                        #
//#                                                                        #
//##########################################################################
#ifndef Q_DATAUTILITY_HEADER
#define Q_DATAUTILITY_HEADER
#include <ccPointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

class AxLineSegment
{
public:
	CCVector3 P0, P1;
	
	AxLineSegment(){}
	~AxLineSegment(){}
	AxLineSegment Compute()
	{

	}
	double Distance()
	{
		double d = (P1 - P0).norm();
		return d;
	}
	CCVector3 Direction()
	{
		return P1 - P0;
	}

};

inline void CC2PCL_PointCloud(ccPointCloud& theCloud, pcl::PointCloud< pcl::PointXYZ> &pc)
{
	unsigned pointCount = theCloud.size();
	pc.resize(pointCount);

	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3 *P = theCloud.getPoint(i);
		pc.at(i).x = P->x;
		pc.at(i).y = P->y;
		pc.at(i).z = P->z;
	}
}

inline void CCVec2PCL_PointCloud(std::vector<CCVector3>& theCloud, pcl::PointCloud< pcl::PointXYZ> &pc)
{
	unsigned pointCount = theCloud.size();
	pc.resize(pointCount);

	for (unsigned i = 0; i < pointCount; ++i)
	{
		CCVector3 P = theCloud.at(i);
		pc.at(i).x = P.x;
		pc.at(i).y = P.y;
		pc.at(i).z = P.z;
	}
}

inline void CCVec2PCL_PointXYZVec(std::vector<CCVector3>& theCloud, std::vector< pcl::PointXYZ> &pc)
{
	unsigned pointCount = theCloud.size();
	pc.resize(pointCount);

	for (unsigned i = 0; i < pointCount; ++i)
	{
		CCVector3 P = theCloud.at(i);
		pc.at(i).x = P.x;
		pc.at(i).y = P.y;
		pc.at(i).z = P.z;
	}
}

inline ccPointCloud* PCL2CC_PointCloud(pcl::PointCloud<pcl::PointXYZ> &ptcl, QString group_name)
{
	ccPointCloud* ccCloud = new ccPointCloud();
	int pointCount = ptcl.size();
	if (!ccCloud->reserve(static_cast<unsigned>(pointCount)))
		return NULL;
	for (size_t i = 0; i < pointCount; ++i)
	{
		CCVector3 P(ptcl.at(i).x, ptcl.at(i).y, ptcl.at(i).z);
		ccCloud->addPoint(P);
	}
	ccCloud->setName(group_name);
}

inline bool SortEigenValuesAndVectors(Eigen::Matrix3d& eigenVectors, Eigen::Vector3d& eigenValues)
{
	if (eigenVectors.cols() < 2 || eigenVectors.cols() != eigenValues.rows())
	{
		assert(false);
		return false;
	}

	unsigned n = eigenVectors.cols();
	for (unsigned i = 0; i < n - 1; i++)
	{
		unsigned maxValIndex = i;
		for (unsigned j = i + 1; j<n; j++)
			if (eigenValues[j] > eigenValues[maxValIndex])
				maxValIndex = j;

		if (maxValIndex != i)
		{
			std::swap(eigenValues[i], eigenValues[maxValIndex]);
			for (unsigned j = 0; j < n; ++j)
				std::swap(eigenVectors(j, i), eigenVectors(j, maxValIndex));
		}
	}

	return true;
}

inline bool SortEigenValuesAndVectors(Eigen::Matrix3f& eigenVectors, Eigen::Vector3f& eigenValues)
{
	if (eigenVectors.cols() < 2 || eigenVectors.cols() != eigenValues.rows())
	{
		assert(false);
		return false;
	}

	unsigned n = eigenVectors.cols();
	for (unsigned i = 0; i < n - 1; i++)
	{
		unsigned maxValIndex = i;
		for (unsigned j = i + 1; j<n; j++)
			if (eigenValues[j] > eigenValues[maxValIndex])
				maxValIndex = j;

		if (maxValIndex != i)
		{
			std::swap(eigenValues[i], eigenValues[maxValIndex]);
			for (unsigned j = 0; j < n; ++j)
				std::swap(eigenVectors(j, i), eigenVectors(j, maxValIndex));
		}
	}

	return true;
}

// cn_PnPoly(): crossing number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  0 = outside, 1 = inside
// This code is patterned after [Franklin, 2000]
inline int cn_PnPoly(Eigen::Vector3f P, std::vector<Eigen::Vector3f>  V, int n)
{
	int    cn = 0;    // the  crossing number counter

	// loop through all edges of the polygon
	for (int i = 0; i<n; i++)
	{    // edge from V[i]  to V[i+1]
		if (((V[i].y() <= P.y()) && (V[i + 1].y() > P.y()))     // an upward crossing
			|| ((V[i].y() > P.y()) && (V[i + 1].y() <= P.y()))) { // a downward crossing
			// compute  the actual edge-ray intersect x-coordinate
			float vt = (float)(P.y() - V[i].y()) / (V[i + 1].y() - V[i].y());
			if (P.x() < V[i].x() + vt * (V[i + 1].x() - V[i].x())) // P.x < intersect
				++cn;   // a valid crossing of y=P.y right of P.x
		}
	}
	return (cn & 1);    // 0 if even (out), and 1 if  odd (in)

}
/// <summary>
/// 由增量X和增量Y所形成的向量的象限角度
/// 区别方位角：方位角以正北方向顺时针
/// |                    |
/// |  /                 |b /
/// | / a                |^/
/// |/_)____象限角       |/______方位角
/// </summary>
/// <param name="x">增量X</param>
/// <param name="y">增量Y</param>
/// <returns>象限角</returns>
inline double GetQuadrantAngle(double x, double y)
{
	double theta = atan(y / x);
	if (x > 0 && y == 0) return 0;
	if (x == 0 && y > 0) return M_PI / 2;
	if (x < 0 && y == 0) return M_PI;
	if (x == 0 && y < 0) return 3 * M_PI / 2;

	if (x > 0 && y > 0) return theta;
	if (x > 0 && y < 0) return M_PI * 2 + theta;
	if (x < 0 && y > 0) return theta + M_PI;
	if (x < 0 && y < 0) return theta + M_PI;
	return theta;
}

//===================================================================
// 点到直线距离（垂直距离）
// dist_Point_to_Line(): get the distance of a point to a line
//     Input:  a Point P and a Line L (in any dimension)
//     Return: the shortest distance from P to L
inline double dist_Point_to_Line(CCVector3 P, CCVector3 Lfrom, CCVector3 Lto)
{
	CCVector3 v = Lto - Lfrom;
	CCVector3 w = P - Lfrom;
	double c1 = w.dot(v);
	double c2 = v.dot(v);
	double b = c1 / c2;
	CCVector3 Pb = Lfrom + b * v;
	return (P - Pb).norm();
}

//===================================================================
// 点到线段距离
//dist_Point_to_Segment(): get the distance of a point to a segment
//     Input:  a Point P and a Segment S (in any dimension)
//     Return: the shortest distance from P to S，返回到线段的最短距离
inline double dist_Point_to_Segment(CCVector3 P, CCVector3 S_P0, CCVector3 S_P1)
{
	CCVector3 v = S_P1 - S_P0;
	CCVector3 w = P - S_P0;
	double c1 = w.dot( v);
	if (c1 <= 0)
		return (P- S_P0).norm();
	double c2 = v.dot( v);
	if (c2 <= c1)
		return (P- S_P1).norm();
	double b = c1 / c2;
	CCVector3 Pb = S_P0 + b * v;
	return (P - Pb).norm();
}

/// <summary>
/// 点是否在共线的线段上
/// 1 = P is inside S;
/// 0 = P is not inside S
/// </returns> 
/// </summary>
/// <param name="P">a point P</param>
/// <param name="S">a collinear segment S</param>
/// <returns></returns>
inline bool  isInSegment(CCVector3 P, AxLineSegment S)
{
	if (S.P0.x != S.P1.x)
	{    // S is not vertical
		if (S.P0.x <= P.x && P.x <= S.P1.x)
			return 1;
		if (S.P0.x >= P.x && P.x >= S.P1.x)
			return 1;
	}
	else
	{
		if (S.P0.y != S.P1.y)
		{
			// S is vertical, so test y coordinate
			if (S.P0.y <= P.y && P.y <= S.P1.y)
				return 1;
			if (S.P0.y >= P.y && P.y >= S.P1.y)
				return 1;
		}
		else
		{
			// S is vertical, so test y coordinate
			if (S.P0.z <= P.z && P.z <= S.P1.z)
				return 1;
			if (S.P0.z >= P.z && P.z >= S.P1.z)
				return 1;
		}
	}
	return 0;
}

/// <summary>
/// 点是否在共线的线段延长上
/// 1是;
/// -1反向
inline int  ExtendedLine(CCVector3 P, AxLineSegment S)
{
	if (S.P0.x != S.P1.x)
	{    // S is not vertical
		double flag = (P.x - S.P0.x) / (S.P1.x - S.P0.x);
		if (flag >= 1)
			return 1;
		if (flag<0)
			return -1;
	}
	else
	{
		if (S.P0.y != S.P1.y)
		{
			// S is vertical, so test y coordinate
			double flag = (P.y - S.P0.y) / (S.P1.y - S.P0.y);
			if (flag >= 1)
				return 1;
			if (flag<0)
				return -1;
		}
		else
		{
			double flag = (P.z - S.P0.z) / (S.P1.z - S.P0.z);
			if (flag >= 1)
				return 1;
			if (flag<0)
				return -1;
		}
	}
	return 0;
}

inline bool isParallize_Segment_to_Segment(AxLineSegment S1, AxLineSegment S2)
{
	CCVector3 u=S1.Direction();
	CCVector3 v=S2.Direction();
	CCVector2 uu(u.x, u.y);
	CCVector2 vv(v.x, v.y);
	//CCVector3 w = S1.source() - S2.source();
	float D = uu.cross(vv);
	if (abs(D)< 0.0000000001)//SMALL_NUM
	{
		return true;
	}
	return false;
}

inline bool isColinearity_Segment_to_Segment(AxLineSegment S1, AxLineSegment S2)
{
	CCVector3 u = S1.Direction();
	CCVector3 v = S2.Direction();
	CCVector2 uu(u.x, u.y);
	CCVector2 vv(v.x, v.y);
	CCVector3 w = S2.P0 - S1.P0;
	CCVector2 ww(w.x, w.y);
	float D = uu.cross(vv);
	float D2 = uu.cross(ww);
	if (abs(D)< 0.001 && abs(D2)< 0.001)//平行且共线
	{
		return true;
	}
	return false;
}

inline CCVector3 projectPointToLine(const CCVector3 &point, const CCVector3 &line_point1, const CCVector3 &line_point2)
{
	CCVector3 line_vector = line_point2 - line_point1;
	return line_point1 + (point - line_point1).dot(line_vector) * line_vector / line_vector.dot(line_vector);
}

inline CCVector3 projectPointToLine(const CCVector3 &point, AxLineSegment &line_)
{
	CCVector3 line_vector = line_.Direction();
	CCVector3 line_point1 = line_.P0;
	CCVector3 line_point2= line_.P1;
	return line_point1 + (point - line_point1).dot(line_vector) * line_vector / line_vector.dot(line_vector);
}

inline int sign(double x)
{
	if (x >= 0)
		return 1;
	else
		return -1;
}
#endif