#include "AxSymmetryP2P.h"


AxSymmetryP2P::AxSymmetryP2P() :max_iterations(30)
, minIter(5)
, max_sym_normal_fit_error( pcl::deg2rad(45.0f))
, min_sym_corresp_distance(0.02)
, max_sym_corresp_reflected_distance(0.005)
, eps(1e-6)
{
}


AxSymmetryP2P::~AxSymmetryP2P()
{
}

void AxSymmetryP2P::setParams(int iteration_, float max_n_fit_error, float min_corresp_dis, float max_corresp_dist)
{
	max_iterations = iteration_;
	max_sym_normal_fit_error = max_n_fit_error;
	min_sym_corresp_distance = min_corresp_dis;
	max_sym_corresp_reflected_distance = max_corresp_dist;
}

bool AxSymmetryP2P::initialize()
{
	if (cloud->size() == 0 || cloud_ds->size() == 0)
		return false;

	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud);
	cloud_mean = pca.getMean().head(3);
	SortEigenValuesAndVectors(pca.getEigenVectors(), pca.getEigenValues());
	symmetry_intial.setOrigin(cloud_mean);
	symmetry_intial.setNormal(pca.getEigenVectors().col(1));//最长的轴
	//symmetry_intial.setNormal(pca.getEigenVectors().col(2));
	return true;
}

void AxSymmetryP2P::setCustomCloudMean(Eigen::Vector3f cloud_mean_)
{
	cloud_mean = cloud_mean_;
}

void AxSymmetryP2P::setPointNormal(Eigen::Vector3f cloud_mean_, Eigen::Vector3f cloud_normal_)
{
	cloud_mean = cloud_mean_;
	symmetry_intial.setOrigin(cloud_mean);
	symmetry_intial.setNormal(cloud_normal_);//最长的轴
}

bool AxSymmetryP2P::detect()
{
	symmetry_refined = symmetry_intial;

	pcl::search::KdTree<pcl::PointXYZ>  search_tree;
	search_tree.setInputCloud(cloud);

	pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;

	//优化对象
	ReflSymRefineFunctorDiff<pcl::PointXYZ> functor;
	functor.cloud_ = cloud;
	functor.cloud_ds_ = cloud_ds;

	//主循环--------------------------------------------------------------------------
	int nrIterations = 0;
	ReflectionalSymmetry symmetry_prev;

	bool done = false;
	while (!done)
	{
		symmetry_prev = symmetry_refined;
		//对应点估计与剔除------------------------------------------------------------------------
		correspondences.clear();//重置correspondences

		//对称平面的参数
		Eigen::Vector3f symmetryOrigin = symmetry_refined.getOrigin();
		Eigen::Vector3f symmetryNormal = symmetry_refined.getNormal();

		//寻找对应点
		for (size_t pointId = 0; pointId < cloud_ds->size(); pointId++)
		{
			Eigen::Vector3f srcPoint = cloud_ds->points[pointId].getVector3fMap();//获取点
			//Eigen::Vector3f srcNormal = cloud_ds->points[pointId].getNormalVector3fMap();//获取点的法向量

			// NOTE: somehow this gives sliiightly worse results than using the rejection below
			//         // If point is too close to the symmetry plane - don't use it as a correspondence
			//         if (std::abs(symmetry.pointSignedDistance(srcPoint)) < 0.01f)
			//           continue;

			//计算对称点和法向量
			Eigen::Vector3f srcPointReflected = symmetry_refined.reflectPoint(srcPoint);
			//Eigen::Vector3f srcNormalRefleclted = symmetry_refined.reflectNormal(srcNormal);//对称法向量

			//搜索最近点
			std::vector<float>  distancesSquared(1);
			std::vector<int>    neighbours(1);
			pcl::PointXYZ searchPoint;
			searchPoint.getVector3fMap() = srcPointReflected;
			//searchPoint.getNormalVector3fMap() = srcNormalRefleclted;
			search_tree.nearestKSearch(searchPoint, 1, neighbours, distancesSquared);

			Eigen::Vector3f tgtPoint = cloud->points[neighbours[0]].getVector3fMap();
			//Eigen::Vector3f tgtNormal = cloud->points[neighbours[0]].getNormalVector3fMap();

			//如果距离along对称轴normal between the points of a symmetric correspondence is too small - reject
			if (std::abs(symmetry_intial.pointSignedDistance(srcPoint) - symmetry_intial.pointSignedDistance(tgtPoint)) < min_sym_corresp_distance)
				continue;

			//距离between the reflected source point and it's nearest neighbor is too big - reject
			if (distancesSquared[0] > max_sym_corresp_reflected_distance * max_sym_corresp_reflected_distance)
				continue;

			//剔除normal error is too high
			/*float error = sym::getReflSymNormalFitError(srcNormal, tgtNormal, symmetry_refined, true);
			if (error > max_sym_normal_fit_error)
				continue;*/

			// If all checks passed - add correspondence
			correspondences.push_back(pcl::Correspondence(pointId, neighbours[0], distancesSquared[0]));
		}

		// Correspondence rejection one to one
		correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

		// Check if there are enough correspondences
		if (correspondences.size() == 0)
			return false;

		//优化------------------------------------------------------------------------
		//构造优化向量
		Eigen::VectorXf x(6);
		x.head(3) = symmetryOrigin;
		x.tail(3) = symmetryNormal;

		// Construct functor object
		functor.correspondences_ = correspondences;

		// Optimize!
		Eigen::LevenbergMarquardt<ReflSymRefineFunctorDiff<pcl::PointXYZ>, float> lm(functor);
		lm.minimize(x);

		// Convert to symmetry
		symmetry_refined = ReflectionalSymmetry(x.head(3), x.tail(3));
		symmetry_refined.setOriginProjected(cloud_mean);

		//检查收敛性和迭代次数------------------------------------------------------------------------
		if (++nrIterations >= max_iterations)
			done = true;

		// Check if symmetry has changed enough
		float angleDiff, distanceDiff;
		symmetry_refined.reflSymDifference(symmetry_prev, cloud_mean, angleDiff, distanceDiff);
		if (angleDiff < pcl::deg2rad(0.05f) && distanceDiff < 0.0001f)
			done = true;
	}
	return true;
}

bool AxSymmetryP2P::detect2()
{
	symmetry_refined = symmetry_intial;

	pcl::search::KdTree<pcl::PointXYZ>  search_tree;
	search_tree.setInputCloud(cloud);

	pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;

	//优化对象
	ReflSymRefineFunctorDiff<pcl::PointXYZ> functor;
	functor.cloud_ = cloud;
	functor.cloud_ds_ = cloud_ds;

	//主循环--------------------------------------------------------------------------
	ReflectionalSymmetry symmetry_prev;

	double oldres = 0;
	double	res = 9e99;
	int totalJump = 0;

	//迭代的主循环
	for (int iterIdx = 0; iterIdx< max_iterations; iterIdx++)
	{
		oldres = res;
		symmetry_prev = symmetry_refined;
		//对应点估计与剔除------------------------------------------------------------------------
		correspondences.clear();//重置correspondences

		//对称平面的参数
		Eigen::Vector3f symmetryOrigin = symmetry_refined.getOrigin();
		Eigen::Vector3f symmetryNormal = symmetry_refined.getNormal();
		double tmpsum = 0;
		int num = 0;
		//寻找对应点
		for (size_t pointId = 0; pointId < cloud_ds->size(); pointId++)
		{
			Eigen::Vector3f srcPoint = cloud_ds->points[pointId].getVector3fMap();//获取点
			//计算对称点和法向量
			Eigen::Vector3f srcPointReflected = symmetry_refined.reflectPoint(srcPoint);
			//Eigen::Vector3f srcNormalRefleclted = symmetry_refined.reflectNormal(srcNormal);//对称法向量

			//搜索最近点
			std::vector<float>  distancesSquared(1);
			std::vector<int>    neighbours(1);
			pcl::PointXYZ searchPoint;
			searchPoint.getVector3fMap() = srcPointReflected;
			//searchPoint.getNormalVector3fMap() = srcNormalRefleclted;
			search_tree.nearestKSearch(searchPoint, 1, neighbours, distancesSquared);

			Eigen::Vector3f tgtPoint = cloud->points[neighbours[0]].getVector3fMap();

			//距离between the reflected source point and it's nearest neighbor is too big - reject
			if (distancesSquared[0] > max_sym_corresp_reflected_distance * max_sym_corresp_reflected_distance)
				continue;
			
			// If all checks passed - add correspondence
			correspondences.push_back(pcl::Correspondence(pointId, neighbours[0], distancesSquared[0]));
		}
		
		// Correspondence rejection one to one
		correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

		// Check if there are enough correspondences
		if (correspondences.size() == 0)
			return false;
		if (correspondences.size() < 5)
			return false;
		for (size_t i = 0; i < correspondences.size(); i++)
		{
			num++;
			tmpsum += sqrt(correspondences[i].distance );
		}
		res = tmpsum / num;
		//优化------------------------------------------------------------------------
		//构造优化向量
		Eigen::VectorXf x(6);
		x.head(3) = symmetryOrigin;
		x.tail(3) = symmetryNormal;

		// Construct functor object
		functor.correspondences_ = correspondences;

		// Optimize!
		Eigen::LevenbergMarquardt<ReflSymRefineFunctorDiff<pcl::PointXYZ>, float> lm(functor);
		lm.minimize(x);

		// Convert to symmetry
		symmetry_refined = ReflectionalSymmetry(x.head(3), x.tail(3));
		symmetry_refined.setOriginProjected(cloud_mean);

		//满足收敛条件，退出
		if (iterIdx >= minIter)
		{
			if (abs(oldres - res) < eps)
			{
				totalJump++;
				if (totalJump > 5)
				{
					break;
				}
			}
			if (res < eps)
				break;
		}
	}
	return true;
}

//限制对应点的数目
bool AxSymmetryP2P::detectFixNumCorresponding()
{
	symmetry_refined = symmetry_intial;

	pcl::search::KdTree<pcl::PointXYZ>  search_tree;
	search_tree.setInputCloud(cloud);

	pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;

	//优化对象
	ReflSymRefineFunctorDiff<pcl::PointXYZ> functor;
	functor.cloud_ = cloud;
	functor.cloud_ds_ = cloud_ds;

	//主循环--------------------------------------------------------------------------
	ReflectionalSymmetry symmetry_prev;

	double oldres = 0;
	double	res = 9e99;
	int totalJump = 0;

	//迭代的主循环
	for (int iterIdx = 0; iterIdx< max_iterations; iterIdx++)
	{
		oldres = res;
		symmetry_prev = symmetry_refined;
		//对应点估计与剔除------------------------------------------------------------------------
		correspondences.clear();//重置correspondences

		//对称平面的参数
		Eigen::Vector3f symmetryOrigin = symmetry_refined.getOrigin();
		Eigen::Vector3f symmetryNormal = symmetry_refined.getNormal();
		double tmpsum = 0;
		int num = 0;

		size_t iter = 0;
		size_t numMax = cloud_ds->size();
		size_t numSlectedPts = 30;
		std::vector<size_t> nCard(numMax, 0);
		if (numSlectedPts>= numMax)
		{
			numSlectedPts = numMax;
			for (int i = 0; i < numMax; i++)
			{
				nCard[i] = i;
			}
		}
		else
		{
			for (int i = 0; i < numMax; i++)
			{
				nCard[i] = i;
			}
			srand(time(NULL));
			std::random_shuffle(nCard.begin(), nCard.begin() + numMax);
		}
				
		size_t selectedPointIndex = -1;
		do
		{
			selectedPointIndex = nCard[iter]; //could be not random!
			if (selectedPointIndex>0 && selectedPointIndex < numMax)
			{
				Eigen::Vector3f srcPoint = cloud_ds->points[selectedPointIndex].getVector3fMap();//获取点
																					  //计算对称点和法向量
				Eigen::Vector3f srcPointReflected = symmetry_refined.reflectPoint(srcPoint);
				//Eigen::Vector3f srcNormalRefleclted = symmetry_refined.reflectNormal(srcNormal);//对称法向量

				//搜索最近点
				std::vector<float>  distancesSquared(1);
				std::vector<int>    neighbours(1);
				pcl::PointXYZ searchPoint;
				searchPoint.getVector3fMap() = srcPointReflected;
				//searchPoint.getNormalVector3fMap() = srcNormalRefleclted;
				search_tree.nearestKSearch(searchPoint, 1, neighbours, distancesSquared);

				Eigen::Vector3f tgtPoint = cloud->points[neighbours[0]].getVector3fMap();

				//距离between the reflected source point and it's nearest neighbor is too big - reject
				if (distancesSquared[0] > max_sym_corresp_reflected_distance * max_sym_corresp_reflected_distance)
					continue;

				// If all checks passed - add correspondence
				correspondences.push_back(pcl::Correspondence(selectedPointIndex, neighbours[0], distancesSquared[0]));
			}
		} while (iter++ < numSlectedPts-1);

		//寻找对应点
		/*for (size_t pointId = 0; pointId < cloud_ds->size(); pointId++)
		{
			
		}*/

		// Correspondence rejection one to one
		correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);

		// Check if there are enough correspondences
		if (correspondences.size() == 0)
			return false;
		if (correspondences.size() < 5)
			return false;
		for (size_t i = 0; i < correspondences.size(); i++)
		{
			num++;
			tmpsum += sqrt(correspondences[i].distance);
		}
		res = tmpsum / num;
		//优化------------------------------------------------------------------------
		//构造优化向量
		Eigen::VectorXf x(6);
		x.head(3) = symmetryOrigin;
		x.tail(3) = symmetryNormal;

		// Construct functor object
		functor.correspondences_ = correspondences;

		// Optimize!
		Eigen::LevenbergMarquardt<ReflSymRefineFunctorDiff<pcl::PointXYZ>, float> lm(functor);
		lm.minimize(x);

		// Convert to symmetry
		symmetry_refined = ReflectionalSymmetry(x.head(3), x.tail(3));
		symmetry_refined.setOriginProjected(cloud_mean);

		//满足收敛条件，退出
		if (iterIdx >= minIter)
		{
			if (abs(oldres - res) < eps)
			{
				totalJump++;
				if (totalJump > 5)
				{
					break;
				}
			}
			if (res < eps)
				break;
		}
	}
	return true;
}