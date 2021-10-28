#pragma once
#ifndef AXSYMMETRYP2P_HEADER
#define AXSYMMETRYP2P_HEADER
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include "refinement_base_functor.hpp"
#include <pcl/common/angles.h>

#include "geometry.hpp"
#include "../dataUtility.h"

template <typename PointT>
struct ReflSymRefineFunctor : BaseFunctor<float>
{
	ReflSymRefineFunctor()  {};

	int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
	{
		ReflectionalSymmetry symmetry(x.head(3), x.tail(3));
		// Compute fitness
		for (size_t i = 0; i < this->correspondences_.size(); i++)
		{
			int srcPointIndex = correspondences_[i].index_query;
			int tgtPointIndex = correspondences_[i].index_match;

			Eigen::Vector3f srcPoint = cloud_ds_->points[srcPointIndex].getVector3fMap();
			//Eigen::Vector3f srcNormal = cloud_ds_->points[srcPointIndex].getNormalVector3fMap();
			Eigen::Vector3f tgtPoint = cloud_->points[tgtPointIndex].getVector3fMap();
			//Eigen::Vector3f tgtNormal = cloud_->points[tgtPointIndex].getNormalVector3fMap();

			//对称目标点和法向量
			Eigen::Vector3f tgtPointReflected = symmetry.reflectPoint(tgtPoint);
			//Eigen::Vector3f tgtNormalReflected = symmetry.reflectNormal(tgtNormal);

			//fvec(i) = std::abs(utl::pointToPlaneSignedDistance<float>(srcPoint, tgtPointReflected, tgtNormalReflected));
			fvec(i) = std::abs(utl::pointToPointDistance<float>(srcPoint, tgtPointReflected));
		}
		return 0;
	}

	//输入点云
	typename pcl::PointCloud<PointT>::ConstPtr cloud_;

	//降采样后点云
	typename pcl::PointCloud<PointT>::ConstPtr cloud_ds_;

	/** \brief Input correspondences. */
	pcl::Correspondences correspondences_;

	/** \brief Dimensionality of the optimization parameter vector. */
	int inputs() const { return 6; }

	/** \brief Number of points. */
	//     int values() const { return this->cloud_ds_->size(); }
	int values() const { return this->correspondences_.size(); }
};

template <typename PointT>
struct ReflSymRefineFunctorDiff : Eigen::NumericalDiff<ReflSymRefineFunctor<PointT> > {};

/** \brief Class representing a reflectional symmetry in 3D space. A symmetry
* is represented as a 3D plane.
*/
class ReflectionalSymmetry
{
public:
	ReflectionalSymmetry(): origin_(Eigen::Vector3f::Zero())
		, normal_(Eigen::Vector3f::Zero())
	{};

	/** \brief A constructor from origin point and normal
	*  \param[in] origin origin point
	*  \param[in] normal symmetry plane normal
	*/
	ReflectionalSymmetry(const Eigen::Vector3f &origin, const Eigen::Vector3f &normal)
		: origin_(origin)
		, normal_(normal.normalized())
	{ };

	/** \brief A constructor from plane coefficients (ax + by + cz = d)
	*  \param[in] plane_coefficients plane coefficients
	*/
	//ReflectionalSymmetry(const Eigen::Vector4f &plane_coefficients)
	//{
	//	utl::planeCoefficientsToPointNormal<float>(plane_coefficients, origin_, normal_);
	//};

	/** \brief Get the normal vector describing the symmetry
	*  \return normal of the symmetry plane
	*/
	Eigen::Vector3f getNormal() const { return normal_; }

	/** \brief Get the origin point of the symmetry
	*  \return symmetry origin point
	*/
	Eigen::Vector3f getOrigin() const { return origin_; }

	/** \brief Set the origin point of the symmetry
	*  \param[in] origin symmetry origin point
	*/
	void setOrigin(const Eigen::Vector3f origin)
	{
		origin_ = origin;
	}

	void setNormal(const Eigen::Vector3f normal)
	{
		normal_ = normal;
	}

	Eigen::Vector3f projectPoint(const Eigen::Vector3f &point)  const
	{
		return utl::projectPointToPlane<float>(point, origin_, normal_);
	};

	void setOriginProjected(const Eigen::Vector3f &point)
	{
		origin_ = projectPoint(point);
	};

	inline void fromTwoPoints(const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
	{
		Eigen::Vector3f origin = (point1 + point2) / 2;
		Eigen::Vector3f normal = point1 - point2;
		normal.normalize();
		*this = ReflectionalSymmetry(origin, normal);
	}
	inline	ReflectionalSymmetry transform(const Eigen::Affine3f &transform) const
	{
		return ReflectionalSymmetry(transform * origin_, transform.rotation() * normal_);
	}

	Eigen::Vector4f getPlaneCoefficients() const
	{
		Eigen::Vector4f plane;
		plane.head(3) = normal_;
		plane[3] = -origin_.dot(normal_);
		return plane;
	}

	float pointSignedDistance(const Eigen::Vector3f &point)  const
	{
		return utl::pointToPlaneSignedDistance<float>(point, origin_, normal_);
	};

	/** \brief Calculate the difference between two reflectional symmetries. Two
	* measures are calculated:
	* 1) Angle between symmetry planes
	* 2) Distance between the projections of a reference point onto the two
	*    symmetry planes
	*  \param[in]  symmetry_other second symmetry
	*  \param[in]  reference_point reference point
	*  \param[out]  angle     angle between symmetry planes
	*  \param[out]  distance  distance between projections of the reference point
	*  \note maximum angle between two symmetry axes is 90 degrees
	*/
	inline void reflSymDifference(const ReflectionalSymmetry &symmetry_other, const Eigen::Vector3f &reference_point, float &angle, float &distance)
	{
		// Get angle between normals
		angle = utl::lineLineAngle<float>(normal_, symmetry_other.getNormal());
		Eigen::Vector3f refPointProj1 = projectPoint(reference_point);
		Eigen::Vector3f refPointProj2 = symmetry_other.projectPoint(reference_point);
		distance = utl::pointToPointDistance<float>(refPointProj1, refPointProj2);
	}

	/** \brief Reflect a point around a symmetry plane
	*  \param[in] point original cloud
	*  \return reflected point
	*/
	inline Eigen::Vector3f reflectPoint(const Eigen::Vector3f &point) const
	{
		return (point - 2 * normal_ * (normal_.dot(point - origin_)));
	}

	/** \brief Reflect a normal around a symmetry plane
	*  \param[in] normal original cloud
	*  \return reflected normal
	*/
	inline	Eigen::Vector3f reflectNormal(const Eigen::Vector3f &normal) const
	{
		return (normal - 2 * (normal.dot(normal_) * normal_));
	}

	/** \brief Reflect a given pointcloud around a symmetry plane
	*  \param[in] cloud_in original cloud
	*  \param[in] cloud_out reflected cloud
	*/
	template <typename PointT>
	inline void reflectCloud(const typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out)  const
	{
		cloud_out = cloud_in;
		for (size_t pointId = 0; pointId < cloud_out.size(); pointId++)
			cloud_out.points[pointId].getVector3fMap() = reflectPoint(cloud_out.points[pointId].getVector3fMap());
	}

	/** \brief Reflect a given pointcloud around a symmetry plane
	*  \param[in] cloud_in original cloud
	*  \param[in] cloud_out reflected cloud
	*/
	template <typename PointT>
	inline void reflectCloudWithNormals(const typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out) const
	{
		cloud_out = cloud_in;
		for (size_t pointId = 0; pointId < cloud_in.size(); pointId++)
		{
			cloud_out.points[pointId].getVector3fMap() = reflectPoint(cloud_out.points[pointId].getVector3fMap());
			cloud_out.points[pointId].getNormalVector3fMap() = reflectNormal(cloud_out.points[pointId].getNormalVector3fMap());
		}
	}
 

protected:
	Eigen::Vector3f origin_;  ///< Point belonging to the symmetry plane. Symmetry is visualized around this point.
	Eigen::Vector3f normal_;  ///< Normal of the symmetry plane. Note that normal is always reoriented such that it's x coordinate is non-negative
};


class AxSymmetryP2P
{
public:
	AxSymmetryP2P();
	~AxSymmetryP2P();

	void setParams(int iteration_, float max_n_fit_error, float min_corresp_dis, float max_corresp_dist);

	bool detect();
	bool detect2();
	//限制对应点的数目
	bool detectFixNumCorresponding();
	bool initialize();
	void setCustomCloudMean(Eigen::Vector3f cloud_mean);
	void setPointNormal(Eigen::Vector3f cloud_mean_, Eigen::Vector3f cloud_normal_);
public:
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ds;
	
	int max_iterations;
	float max_sym_normal_fit_error ;
	float min_sym_corresp_distance ;
	float max_sym_corresp_reflected_distance ;
	ReflectionalSymmetry symmetry_refined;
protected:
	Eigen::Vector3f cloud_mean;
	pcl::Correspondences correspondences;
	ReflectionalSymmetry symmetry_intial;

public:
	int minIter;
	float eps;
};
#endif

