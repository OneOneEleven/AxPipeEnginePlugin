#pragma once
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>

class CVertex
{
public:
	Eigen::Vector3f mPoint;
	std::vector<int> neighbors;
	std::vector<int> original_neighbors;

	bool bIsOriginal;
	int m_index;

	bool is_fixed_sample; //feature points (blue color) 
	bool is_skel_ignore;

	/* for skeletonization */
	double eigen_confidence;
	Eigen::Vector3f eigen_vector0; //Associate with the biggest eigen value
	Eigen::Vector3f eigen_vector1; // Also use for remember last better virtual point
						   //Point3f eigen_vector2; //The smallest eigen value : should be PCA normal N()
	Eigen::Vector3f eigen_vector2_normal;
	bool is_skel_virtual; //in our papaer, we said bridge point instead of virtual point
	bool is_skel_branch;
	bool is_fixed_original;

	double skel_radius; // remember radius for branches

public:
	CVertex() :
		m_index(0),
		bIsOriginal(false),
		is_fixed_sample(false),
		eigen_confidence(0),
		is_skel_branch(false),
		is_skel_ignore(false),
		is_skel_virtual(false),
		is_fixed_original(false),
		eigen_vector0(Eigen::Vector3f(1, 0, 0)),
		eigen_vector1(Eigen::Vector3f(0, 1, 0)),
		skel_radius(-1.0)
	{}
	bool isSample_Moving()
	{
		return (!is_skel_ignore && !is_fixed_sample);
		//return (!is_skel_ignore && !is_fixed_sample && !is_skel_branch);
	}
	bool isSample_JustMoving()
	{
		return (!is_skel_ignore && !is_fixed_sample);
		//return (!is_skel_ignore && !is_fixed_sample && !is_skel_virtual && !is_skel_branch);
	}
	void remove() //important, some time we don't want to earse points, just remove them
	{
		neighbors.clear();
		original_neighbors.clear();
		is_skel_ignore = true;
		mPoint = Eigen::Vector3f(0, 0, 0);
	}
};

class AxImprovedMeanshift
{
public:
	AxImprovedMeanshift();
	~AxImprovedMeanshift();
	void setInputPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_0);
	void setNumSamplesParameter(bool isNum_, float percent_, unsigned num_);
	void initialRadius();

	void run();
	void downSampling(float percent_);
	void downSamplingFixedNum(int numSlectedPts_);

	void initVertexes();
	double wlopIterate();
	void computeAverageTerm();
	void computeRepulsionTerm();
	void computeEigenWithTheta();
	void removeTooClosePoints();
	void eigenConfidenceSmoothing();
	void eigenThresholdIdentification();
	void computeEigen(std::vector<CVertex>& samples_);
	std::vector<CVertex> getResults();
protected:
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_;

	pcl::PointCloud<pcl::PointXYZ>::Ptr sub_samples;
	std::vector<CVertex> samples;

	std::vector<double> samples_density;
	std::vector<double> original_density;
	std::vector<Eigen::Vector3f> repulsion;
	std::vector<double>  repulsion_weight_sum;

	std::vector<Eigen::Vector3f> average;
	std::vector<double>  average_weight_sum;

	double error_x;
	double average_power;
	double repulsion_power;// "Repulsion Power"
	bool need_density;// Need Compute Density;
	double radius;// CGrid Radius;
	double fix_original_weight;// "Fix Original Weight";
	double h_gaussian_para; //H Gaussian Para
	double mu_max;
	double mu_min;
	double currentMoveError;// "Current Movement Error"
	double stopAndGrowError; //para->getDouble("Stop And Grow Error")
	int iterate_time_in_one_stage;
	int maxIterateTimes;// para->getDouble("Max Iterate Time")

	double init_para = 1.0;
	double TAG_STOP_RADIUS;
	//采样参数
	bool isNumSample;
	unsigned numofSamples;
	float percent;
	float speed;
};

