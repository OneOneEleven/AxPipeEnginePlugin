#pragma once
#ifndef AXPIPELINETRACKING_HEADER
#define AXPIPELINETRACKING_HEADER

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include "dataUtility.h"
#include <vector>
#include <algorithm>
#include "GlobeFunction.h"
//#include "AxVertex.h"
//#include "AxBranch.h"
//#include "shapefil.h"
#include "Symmetry/AxSymmetryP2P.h"
#include <QDebug>
#include <omp.h>

class AxPipeLineTracking
{
public:
	AxPipeLineTracking();
	~AxPipeLineTracking();
	void setInputPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_0);
	
	void compute();
	void computeEx();
	bool doFitCircleTest(Eigen::Vector3f center, Eigen::Vector3f normal_sys, std::vector<int> nn_indices, std::vector<float> nn_dists, pcl::PointXYZI& pt_0);
	void downSampling(float percent_);
	void computeRaduis();
	//设置参数拟合圆的迭代次数，距离容差，切片厚度
	void setParameters(int iter_,double threshold_, double thickness_);
	void setParametersRaduis(double raduis_max_, double raduis_min_);
	void setParametersSysmmetry(int iteration_, float max_n_fit_error, float min_corresp_dis, float max_corresp_dist);

	int saveBranchReferencePointCloud(QString  _saveFilePath);

	pcl::PointCloud<pcl::PointXYZI>::Ptr getFinalSamples();
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_ds;

	int circle_iter_max;
	double circle_threshold_dist;
	double raduis_max;
	double raduis_min;
	double thickness;
	double Branches_Search_Angle;
	pcl::PointCloud<pcl::PointXYZI>::Ptr output_;

	float reject_min_angle;
	float reject_max_angle;
protected:
	int max_sym_iterations;
	float max_sym_normal_fit_error;
	float min_sym_corresp_distance;
	float max_sym_corresp_reflected_distance;

};
#endif

