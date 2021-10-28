#include "AxPipeLineTracking.h"



AxPipeLineTracking::AxPipeLineTracking() :Branches_Search_Angle(150)
, circle_iter_max(100)
, circle_threshold_dist(0.01)
, thickness(0.01)
, raduis_max(1.0)
, raduis_min(0.1)
, max_sym_iterations(30)
, max_sym_normal_fit_error(pcl::deg2rad(45.0f))
, min_sym_corresp_distance(0.2)
, max_sym_corresp_reflected_distance(0.005)
, reject_min_angle(60.0)
, reject_max_angle(180.0)
{
}

AxPipeLineTracking::~AxPipeLineTracking()
{
}


void AxPipeLineTracking::computeRaduis()
{
	if (input_->points.size() == 0 || input_ds->points.size() == 0)
		return;
	/*---------------构造KD树-------------*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(input_);
	if (!output_)
	{
		output_.reset(new pcl::PointCloud<pcl::PointXYZI>);
	}
	float raduis = raduis_max;
	for (int idx = 0; idx < static_cast<int> (input_ds->size()); ++idx)
	{
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		const pcl::PointXYZ pt = input_ds->at(idx);
		const float & cx = pt.x;
		const float & cy = pt.y;
		const float & cz = pt.z;
		if (tree->radiusSearch(pt, raduis, nn_indices, nn_dists) == 0)//有错误，程序无法进入
		{
			continue;
		}
		//判断对称轴，是否对称
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_neiber_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int k = 1; k < nn_indices.size(); k++)
		{
			const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
			pcl_neiber_cloud->push_back(source_pt);
		}
		AxSymmetryP2P sysmmetry_detect;
		sysmmetry_detect.cloud = pcl_neiber_cloud;
		sysmmetry_detect.cloud_ds = pcl_neiber_cloud;//未进行降采样
		sysmmetry_detect.setParams(max_sym_iterations, max_sym_normal_fit_error, min_sym_corresp_distance, max_sym_corresp_reflected_distance);//设置对称轴检测参数
		bool flag0 = sysmmetry_detect.initialize();
		Eigen::Vector3f current_pt;
		current_pt[0] = cx;//当前点
		current_pt[1] = cy;
		current_pt[2] = cz;
		sysmmetry_detect.setCustomCloudMean(current_pt);
		if (!flag0)
		{
			continue;
		}
		bool flag=sysmmetry_detect.detect();
		if (!flag)
		{
			continue;
		}
		ReflectionalSymmetry sysmmetry_refined = sysmmetry_detect.symmetry_refined;
		Eigen::Vector3f origin_sys = sysmmetry_refined.getOrigin();
		Eigen::Vector3f normal_sys = sysmmetry_refined.getNormal();
		//获取近邻点，获取走向线
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_Direction_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int k = 1; k < nn_indices.size(); k++)
		{
			const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
			const float & sx = source_pt.x;
			const float & sy = source_pt.y;
			const float & sz = source_pt.z;
			Eigen::Vector3f vpt(sx, sy, sz);
			Eigen::Vector3f center(cx, cy, cz);
			//到对称平面的距离
			double d = utl::pointToPlaneSignedDistance(vpt, center, normal_sys);//点乘
			if (d <= thickness)
			{
				pcl_t_Direction_cloud->push_back(source_pt);
			}
		}
		//就是均值和协方差阵
		EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
		Eigen::Vector4f xyz_centroid;
		if (pcl::computeMeanAndCovarianceMatrix(*pcl_t_Direction_cloud, covariance_matrix, xyz_centroid) == 0)
		{
			continue;
		}
		EIGEN_ALIGN16 Eigen::Matrix3f eigen_vector3;
		EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
		//计算特征值和特征向量
		pcl::eigen33(covariance_matrix, eigen_vector3, eigen_values);
		SortEigenValuesAndVectors(eigen_vector3, eigen_values);
		double eig_val1 = eigen_values[0];//最大
		double eig_val2 = eigen_values[1];
		double eig_val3 = eigen_values[2];
		Eigen::Vector3f direction;
		direction[0] = eigen_vector3(0, 0);//最小的法向量
		direction[1] = eigen_vector3(1, 0);
		direction[2] = eigen_vector3(2, 0);

		//获取近邻点，获取切面
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int k = 1; k < nn_indices.size(); k++)
		{
			const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
			//到垂直剖切面的距离
			const float & sx = source_pt.x;
			const float & sy = source_pt.y;
			const float & sz = source_pt.z;
			Eigen::Vector3f pt1(sx, sy, sz);
			Eigen::Vector3f center(cx, cy, cz);
			//double d = abs(direction[0] * (cx - sx) + direction[1] * (cy - sy) + direction[2] * (cz - sz));
			double d = utl::pointToPlaneSignedDistance(pt1, center, direction);//点乘
			if (d <= thickness)
			{		
				Eigen::Vector3f vpt_back = utl::projectPointToPlane<float>(pt1, center, direction);
				pcl::PointXYZ pt_0;
				pt_0.x = vpt_back.x();
				pt_0.y = vpt_back.y();
				pt_0.z = vpt_back.z();
				pcl_t_cloud->push_back(pt_0);
				//output_->push_back(source_pt);
			}
		}
		if (pcl_t_cloud->size() < 5)
		{
			continue;
		}
		//计算当前采样点的主成分，判断当前点弧面的对称性和比例
		pcl::ModelCoefficients::Ptr coefficients_circle3d(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers_Circle3D(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CIRCLE3D);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(circle_iter_max);
		seg.setDistanceThreshold(circle_threshold_dist);
		seg.setInputCloud(pcl_t_cloud);
		seg.segment(*inliers_Circle3D, *coefficients_circle3d);
		float percent = static_cast<float>(inliers_Circle3D->indices.size()) / pcl_t_cloud->size();
		float circle_radius = coefficients_circle3d->values[3];//半径
		if (percent > 0.75 && circle_radius< 2 * raduis)
		{
			//计算圆心
			const float & circle_x = coefficients_circle3d->values[0];
			const float & circle_y = coefficients_circle3d->values[1];
			const float & circle_z = coefficients_circle3d->values[2];
			pcl::PointXYZI pt_0;
			pt_0.x = circle_x;
			pt_0.y = circle_y;
			pt_0.z = circle_z;
			pt_0.intensity = circle_radius;
			output_->push_back(pt_0);		
		}
		else if (percent > 0.75 && circle_radius> 2 * raduis)
		{
			for (int k = 1; k < pcl_t_cloud->size(); k++)
			{
				pcl::PointXYZ pt_0;
				pt_0.x = pcl_t_cloud->at(k).x;
				pt_0.y = pcl_t_cloud->at(k).y;
				pt_0.z = pcl_t_cloud->at(k).z;
				//output_->push_back(pt_0);
			}
			
		}
	}
}

//执行管道中轴点云提取
void AxPipeLineTracking::compute()
{
	if (input_->points.size() == 0 || input_ds->points.size() == 0)
		return;
	/*---------------构造KD树-------------*/
	try
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(input_);
		if (!output_)
		{
			output_.reset(new pcl::PointCloud<pcl::PointXYZI>);
		}
		output_->resize(input_ds->size());
		float raduis = raduis_max;
#pragma omp parallel 
		{
#pragma omp for
			for (long idx = 0; idx < input_ds->size(); ++idx)
			{
				std::vector<int> nn_indices;
				std::vector<float> nn_dists;
				const pcl::PointXYZ pt = input_ds->at(idx);
				const float & cx = pt.x;
				const float & cy = pt.y;
				const float & cz = pt.z;
				if (tree->radiusSearch(pt, raduis, nn_indices, nn_dists) == 0)//有错误，程序无法进入
				{
					continue;
				}
				if (nn_indices.size() < 10)//近邻点过少，跳过该点
				{
					continue;
				}
				//判断对称轴，是否对称
				pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_neiber_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				for (size_t k = 1; k < nn_indices.size(); k++)
				{
					const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
					pcl_neiber_cloud->push_back(source_pt);
				}
				Eigen::Vector4f centroid;  //质心 
				pcl::compute3DCentroid(*pcl_neiber_cloud, centroid); //估计质心的坐标
				Eigen::Vector3f G(centroid.x(), centroid.y(), centroid.z());
				Eigen::Vector3f queryPoint(cx, cy, cz);
				double	d = std::abs((G - queryPoint).norm()) / raduis;
				if (d < 0.05)
				{
					continue;
				}
				AxSymmetryP2P sysmmetry_detect;
				sysmmetry_detect.cloud = pcl_neiber_cloud;
				sysmmetry_detect.cloud_ds = pcl_neiber_cloud;//未进行降采样
				sysmmetry_detect.setParams(max_sym_iterations, max_sym_normal_fit_error, min_sym_corresp_distance, max_sym_corresp_reflected_distance);//设置对称轴检测参数
				bool flag0 = sysmmetry_detect.initialize();
				Eigen::Vector3f current_pt;
				current_pt[0] = cx;//当前点
				current_pt[1] = cy;
				current_pt[2] = cz;
				sysmmetry_detect.setCustomCloudMean(current_pt);
				if (!flag0)
				{
					continue;
				}
				bool flag = sysmmetry_detect.detect2();
				//希望加速对称检测速度
				//bool flag = sysmmetry_detect.detectFixNumCorresponding();
				if (!flag)
				{
					continue;
				}
				ReflectionalSymmetry sysmmetry_refined = sysmmetry_detect.symmetry_refined;
				Eigen::Vector3f origin_sys = sysmmetry_refined.getOrigin();
				Eigen::Vector3f normal_sys = sysmmetry_refined.getNormal();
				//获取近邻点，获取走向线
				pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_Direction_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				for (size_t k = 1; k < nn_indices.size(); k++)
				{
					const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
					const float & sx = source_pt.x;
					const float & sy = source_pt.y;
					const float & sz = source_pt.z;
					Eigen::Vector3f vpt(sx, sy, sz);
					Eigen::Vector3f center(cx, cy, cz);
					//到对称平面的距离
					double d = utl::pointToPlaneSignedDistance(vpt, center, normal_sys);//点乘
					if (d <= thickness)
					{
						pcl_t_Direction_cloud->push_back(source_pt);
					}
				}
				if (pcl_t_Direction_cloud->size() < 5)
				{
					continue;
				}
				//就是均值和协方差阵
				EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
				Eigen::Vector4f xyz_centroid;
				if (pcl::computeMeanAndCovarianceMatrix(*pcl_t_Direction_cloud, covariance_matrix, xyz_centroid) == 0)
				{
					continue;
				}
				EIGEN_ALIGN16 Eigen::Matrix3f eigen_vector3;
				EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
				//计算特征值和特征向量
				pcl::eigen33(covariance_matrix, eigen_vector3, eigen_values);
				SortEigenValuesAndVectors(eigen_vector3, eigen_values);
				double eig_val1 = eigen_values[0];//最大
				double eig_val2 = eigen_values[1];
				double eig_val3 = eigen_values[2];
				Eigen::Vector3f direction;
				direction[0] = eigen_vector3(0, 0);//最长的向量
				direction[1] = eigen_vector3(1, 0);
				direction[2] = eigen_vector3(2, 0);

				//获取近邻点，获取垂向切面
				pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				std::vector<float> dist_t;
				for (size_t k = 1; k < nn_indices.size(); k++)
				{
					const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
					float dist = std::sqrt(nn_dists[k]);//注意NN近邻搜索的是平方距离
					//到垂直剖切面的距离
					const float & sx = source_pt.x;
					const float & sy = source_pt.y;
					const float & sz = source_pt.z;
					Eigen::Vector3f pt1(sx, sy, sz);
					Eigen::Vector3f center(cx, cy, cz);
					//double d = abs(direction[0] * (cx - sx) + direction[1] * (cy - sy) + direction[2] * (cz - sz));
					double d = utl::pointToPlaneSignedDistance(pt1, center, direction);//点乘
					if (d <= thickness)
					{
						Eigen::Vector3f vpt_back = utl::projectPointToPlane<float>(pt1, center, direction);
						pcl::PointXYZ pt_0;
						pt_0.x = vpt_back.x();
						pt_0.y = vpt_back.y();
						pt_0.z = vpt_back.z();
						pcl_t_cloud->push_back(pt_0);
						dist_t.push_back(dist);
						//output_->push_back(source_pt);
					}
				}
				if (pcl_t_cloud->size() < 5)//拟合圆的切片点太少，跳出
				{
					continue;
				}
				//pcl::ModelCoefficients::Ptr	coefficients_line(new pcl::ModelCoefficients);
				//pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);
				//pcl::SACSegmentation<pcl::PointXYZ> seg;
				//seg.setOptimizeCoefficients(true);
				//seg.setModelType(pcl::SACMODEL_LINE);
				//seg.setMethodType(pcl::SAC_RANSAC);
				//seg.setMaxIterations(circle_iter_max);
				//seg.setDistanceThreshold(circle_threshold_dist);
				//seg.setInputCloud(pcl_t_cloud);
				//seg.segment(*inliers_line, *coefficients_line);
				//float percent = static_cast<float>(inliers_line->indices.size()) / pcl_t_cloud->size();
				//if (percent > 0.85)
				//{
				//	continue;
				//}
				//计算当前采样点的主成分，判断当前点弧面的对称性和比例
				pcl::ModelCoefficients::Ptr coefficients_circle3d(new pcl::ModelCoefficients);
				
				pcl::PointIndices::Ptr inliers_Circle3D(new pcl::PointIndices);
				pcl::SACSegmentation<pcl::PointXYZ> seg_circle;
				seg_circle.setOptimizeCoefficients(true);
				seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
				seg_circle.setMethodType(pcl::SAC_RANSAC);
				seg_circle.setMaxIterations(circle_iter_max);
				seg_circle.setDistanceThreshold(circle_threshold_dist);
				seg_circle.setInputCloud(pcl_t_cloud);
				seg_circle.segment(*inliers_Circle3D, *coefficients_circle3d);
				float percent_circle = static_cast<float>(inliers_Circle3D->indices.size()) / pcl_t_cloud->size();
				float circle_radius = coefficients_circle3d->values[3];//半径
				if (percent_circle > 0.85 && circle_radius < 2 * raduis)
				{
					//计算圆心
					const float & circle_x = coefficients_circle3d->values[0];
					const float & circle_y = coefficients_circle3d->values[1];
					const float & circle_z = coefficients_circle3d->values[2];
					pcl::PointXYZI pt_0;
					pt_0.x = circle_x;
					pt_0.y = circle_y;
					pt_0.z = circle_z;
					pt_0.intensity = circle_radius;
					//增加角度判断，实现自适应半径，及去噪声
					float farPt = dist_t[dist_t.size() - 1];
					float angle_sin = farPt / (2 * circle_radius);//sin值，两条边的比值
					float min_ang_sin = sin(0.5*reject_min_angle / 180 * M_PI);
					float max_ang_sin = sin(0.5*reject_max_angle / 180 * M_PI);
					if (angle_sin > min_ang_sin && angle_sin < max_ang_sin)
					{
						//output_->push_back(pt_0);
						(*output_)[idx] = pt_0;
					}
				}
				
			}
		}

		//剔除无效值
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		for (long idx = 0; idx < output_->size(); ++idx)
		{
			pcl::PointXYZI pt = output_->at(idx);
			if (!pcl_isfinite(pt.x))//pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
			{
				continue;
			}
			if (pt.x == 0 && pt.y == 0 && pt.z == 0)
			{
				continue;
			}
			pcl_tmp_cloud->push_back(pt);
		}
		output_ = pcl_tmp_cloud;
	}
	catch (...)
	{
		throw 1;
	}
}

void AxPipeLineTracking::computeEx()
{
	if (input_->points.size() == 0 || input_ds->points.size() == 0)
		return;
	/*---------------构造KD树-------------*/
	try
	{
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(input_);
		if (!output_)
		{
			output_.reset(new pcl::PointCloud<pcl::PointXYZI>);
		}
		output_->resize(input_ds->size());
		float raduis = raduis_max;
#pragma omp parallel 
		{
#pragma omp for
			for (long idx = 0; idx < input_ds->size(); ++idx)
			{
				std::vector<int> nn_indices;
				std::vector<float> nn_dists;
				const pcl::PointXYZ pt = input_ds->at(idx);
				const float & cx = pt.x;
				const float & cy = pt.y;
				const float & cz = pt.z;
				if (tree->radiusSearch(pt, raduis, nn_indices, nn_dists) == 0)//有错误，程序无法进入
				{
					continue;
				}
				if (nn_indices.size() < 10)//近邻点过少，跳过该点
				{
					continue;
				}
				//判断对称轴，是否对称
				pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_neiber_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				for (size_t k = 1; k < nn_indices.size(); k++)
				{
					const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
					pcl_neiber_cloud->push_back(source_pt);
				}
				Eigen::Vector4f centroid;  //质心 
				pcl::compute3DCentroid(*pcl_neiber_cloud, centroid); //估计质心的坐标
				Eigen::Vector3f G(centroid.x(), centroid.y(), centroid.z());
				Eigen::Vector3f queryPoint(cx, cy, cz);//当前点
				double	d = std::abs((G - queryPoint).norm()) / raduis;
				if (d < 0.05)
				{
					continue;
				}
				AxSymmetryP2P sysmmetry_detect;
				sysmmetry_detect.cloud = pcl_neiber_cloud;
				sysmmetry_detect.cloud_ds = pcl_neiber_cloud;//未进行降采样
				sysmmetry_detect.setParams(max_sym_iterations, max_sym_normal_fit_error, min_sym_corresp_distance, max_sym_corresp_reflected_distance);//设置对称轴检测参数
				//bool flag0 = sysmmetry_detect.initialize();
				pcl::PCA<pcl::PointXYZ> pca;
				pca.setInputCloud(pcl_neiber_cloud);
				SortEigenValuesAndVectors(pca.getEigenVectors(), pca.getEigenValues());
				sysmmetry_detect.setPointNormal(queryPoint, pca.getEigenVectors().col(1));
				bool flagSYS1 = sysmmetry_detect.detect2();
				//希望加速对称检测速度
				//bool flag = sysmmetry_detect.detectFixNumCorresponding();
				if (flagSYS1)
				{
					ReflectionalSymmetry sysmmetry_refined = sysmmetry_detect.symmetry_refined;
					Eigen::Vector3f origin_sys = sysmmetry_refined.getOrigin();
					Eigen::Vector3f normal_sys = sysmmetry_refined.getNormal();
					pcl::PointXYZI pt_0;
					bool isFittedCircle = doFitCircleTest(queryPoint, normal_sys, nn_indices, nn_dists, pt_0);
					if (isFittedCircle)
					{
						(*output_)[idx] = pt_0;
						continue;
					}
					else
					{
						sysmmetry_detect.setPointNormal(queryPoint, pca.getEigenVectors().col(2));
						bool flagSYS12 = sysmmetry_detect.detect2();
						if (flagSYS12)
						{
							ReflectionalSymmetry sysmmetry_refined12 = sysmmetry_detect.symmetry_refined;
							Eigen::Vector3f origin_sys12 = sysmmetry_refined12.getOrigin();
							Eigen::Vector3f normal_sys12 = sysmmetry_refined12.getNormal();
							pcl::PointXYZI pt_02;
							bool isFittedCircle2 = doFitCircleTest(queryPoint, normal_sys12, nn_indices, nn_dists, pt_02);
							if (isFittedCircle2)
							{
								(*output_)[idx] = pt_02;
								continue;
							}
						}
					}
				}
				else
				{
					sysmmetry_detect.setPointNormal(queryPoint, pca.getEigenVectors().col(2));
					bool flagSYS02 = sysmmetry_detect.detect2();
					if (flagSYS02)
					{
						ReflectionalSymmetry sysmmetry_refined02 = sysmmetry_detect.symmetry_refined;
						Eigen::Vector3f origin_sys02 = sysmmetry_refined02.getOrigin();
						Eigen::Vector3f normal_sys02 = sysmmetry_refined02.getNormal();
						pcl::PointXYZI pt_02;
						bool isFittedCircle2 = doFitCircleTest(queryPoint, normal_sys02, nn_indices, nn_dists, pt_02);
						if (isFittedCircle2)
						{
							(*output_)[idx] = pt_02;
							continue;
						}
					}
				}
			}
		}

		//剔除无效值
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
		for (long idx = 0; idx < output_->size(); ++idx)
		{
			pcl::PointXYZI pt = output_->at(idx);
			if (!pcl_isfinite(pt.x))//pcl_isfinite函数返回一个布尔值，检查某个值是不是正常数值
			{
				continue;
			}
			if (pt.x == 0 && pt.y == 0 && pt.z == 0)
			{
				continue;
			}
			pcl_tmp_cloud->push_back(pt);
		}
		output_ = pcl_tmp_cloud;
	}
	catch (...)
	{
		throw 1;
	}
}

bool AxPipeLineTracking::doFitCircleTest(Eigen::Vector3f center, Eigen::Vector3f normal_sys, std::vector<int> nn_indices, std::vector<float> nn_dists, pcl::PointXYZI& pt_0)
{
	float raduis = raduis_max;
	//获取近邻点，获取走向线
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_Direction_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t k = 1; k < nn_indices.size(); k++)
	{
		const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
		const float & sx = source_pt.x;
		const float & sy = source_pt.y;
		const float & sz = source_pt.z;
		Eigen::Vector3f vpt(sx, sy, sz);
		//Eigen::Vector3f center(cx, cy, cz);
		//到对称平面的距离
		double d = utl::pointToPlaneSignedDistance(vpt, center, normal_sys);//点乘
		if (d <= thickness)
		{
			pcl_t_Direction_cloud->push_back(source_pt);
		}
	}
	if (pcl_t_Direction_cloud->size() < 5)
	{
		return false;
	}
	//就是均值和协方差阵
	EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
	Eigen::Vector4f xyz_centroid;
	if (pcl::computeMeanAndCovarianceMatrix(*pcl_t_Direction_cloud, covariance_matrix, xyz_centroid) == 0)
	{
		return false;
	}
	EIGEN_ALIGN16 Eigen::Matrix3f eigen_vector3;
	EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
	//计算特征值和特征向量
	pcl::eigen33(covariance_matrix, eigen_vector3, eigen_values);
	SortEigenValuesAndVectors(eigen_vector3, eigen_values);
	double eig_val1 = eigen_values[0];//最大
	double eig_val2 = eigen_values[1];
	double eig_val3 = eigen_values[2];
	Eigen::Vector3f direction;
	direction[0] = eigen_vector3(0, 0);//最长的向量
	direction[1] = eigen_vector3(1, 0);
	direction[2] = eigen_vector3(2, 0);

	//获取近邻点，获取垂向切面
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<float> dist_t;
	for (size_t k = 1; k < nn_indices.size(); k++)
	{
		const pcl::PointXYZ source_pt = input_->at(nn_indices[k]);
		float dist = std::sqrt(nn_dists[k]);//注意NN近邻搜索的是平方距离
											//到垂直剖切面的距离
		const float & sx = source_pt.x;
		const float & sy = source_pt.y;
		const float & sz = source_pt.z;
		Eigen::Vector3f pt1(sx, sy, sz);
		/*Eigen::Vector3f center(cx, cy, cz);*/
		//double d = abs(direction[0] * (cx - sx) + direction[1] * (cy - sy) + direction[2] * (cz - sz));
		double d = utl::pointToPlaneSignedDistance(pt1, center, direction);//点乘
		if (d <= thickness)
		{
			Eigen::Vector3f vpt_back = utl::projectPointToPlane<float>(pt1, center, direction);
			pcl::PointXYZ pt_0;
			pt_0.x = vpt_back.x();
			pt_0.y = vpt_back.y();
			pt_0.z = vpt_back.z();
			pcl_t_cloud->push_back(pt_0);
			dist_t.push_back(dist);
		}
	}
	if (pcl_t_cloud->size() < 5)//拟合圆的切片点太少，跳出
	{
		return false;
	}
	//计算当前采样点的主成分，判断当前点弧面的对称性和比例
	pcl::ModelCoefficients::Ptr coefficients_circle3d(new pcl::ModelCoefficients);

	pcl::PointIndices::Ptr inliers_Circle3D(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg_circle;
	seg_circle.setOptimizeCoefficients(true);
	seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg_circle.setMethodType(pcl::SAC_RANSAC);
	seg_circle.setMaxIterations(circle_iter_max);
	seg_circle.setDistanceThreshold(circle_threshold_dist);
	seg_circle.setInputCloud(pcl_t_cloud);
	seg_circle.segment(*inliers_Circle3D, *coefficients_circle3d);
	float percent_circle = static_cast<float>(inliers_Circle3D->indices.size()) / pcl_t_cloud->size();
	float circle_radius = coefficients_circle3d->values[3];//半径
	if (percent_circle > 0.85 && circle_radius < 2 * raduis)
	{
		//计算圆心
		const float & circle_x = coefficients_circle3d->values[0];
		const float & circle_y = coefficients_circle3d->values[1];
		const float & circle_z = coefficients_circle3d->values[2];
		//pcl::PointXYZI pt_0;
		pt_0.x = circle_x;
		pt_0.y = circle_y;
		pt_0.z = circle_z;
		pt_0.intensity = circle_radius;
		//增加角度判断，实现自适应半径，及去噪声
		float farPt = dist_t[dist_t.size() - 1];
		float angle_sin = farPt / (2 * circle_radius);//sin值，两条边的比值
		float min_ang_sin = sin(0.5*reject_min_angle / 180 * M_PI);
		float max_ang_sin = sin(0.5*reject_max_angle / 180 * M_PI);
		if (angle_sin > min_ang_sin && angle_sin < max_ang_sin)
		{
			//output_->push_back(pt_0);
			//(*output_)[idx] = pt_0;
			return true;
		}
	}
	return false;
}

void AxPipeLineTracking::setInputPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_0)
{
	input_ = input_0;
}

void AxPipeLineTracking::downSampling(float percent_)
{
	if (!input_)
	{
		return;
	}
	if (input_->points.size() == 0)
		return;
	srand(time(NULL));
	size_t selectedPointIndex = -1;
	size_t iter = 0;
	size_t numSlectedPts = floor(percent_*input_->size());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	input_ds = pcl_t_cloud;

	size_t numMax = input_->size();
	std::vector<size_t> nCard(numMax, 0);
	srand(time(NULL));
	for (int i = 0; i < numMax; i++)
	{
		nCard[i] = i;
	}
	std::random_shuffle(nCard.begin(), nCard.begin() + numMax);

	do
	{
		/*std::random_device rd;
		std::uniform_int_distribution<int> dist(0, input_->size());
		selectedPointIndex = dist(rd);*/
		selectedPointIndex = nCard[iter]; //could be not random!

		//selectedPointIndex = static_cast<size_t>(rand() % input_->size());	//这种方法范围受影响	
		if (selectedPointIndex>0 && selectedPointIndex < numMax)
		{
			pcl::PointXYZ pt = input_->at(selectedPointIndex);
			input_ds->push_back(pt);
		}	
	} while (iter++ < numSlectedPts);
}

void AxPipeLineTracking::setParameters(int iter_, double threshold_, double thickness_)
{
	circle_iter_max = iter_;
	circle_threshold_dist = threshold_;
	thickness = thickness_;
}
//设置自适应半径范围
void AxPipeLineTracking::setParametersRaduis(double raduis_max_, double raduis_min_)
{
	raduis_max = raduis_max_;
	raduis_min = raduis_min_;
}

void AxPipeLineTracking::setParametersSysmmetry(int iteration_, float max_n_fit_error, float min_corresp_dis, float max_corresp_dist)
{
	max_sym_iterations = iteration_;
	max_sym_normal_fit_error = max_n_fit_error;
	min_sym_corresp_distance = min_corresp_dis;
	max_sym_corresp_reflected_distance = max_corresp_dist;
}


int AxPipeLineTracking::saveBranchReferencePointCloud(QString  _saveFilePath)
{
	
	return 1;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr AxPipeLineTracking::getFinalSamples()
{
	return output_;
}


