#include "AxImprovedMeanshift.h"

AxImprovedMeanshift::AxImprovedMeanshift()
{
	//skeleton.addParam(new RichDouble("Repulsion Power", 1.0));
	//skeleton.addParam(new RichDouble("Average Power", 2.0));
	//skeleton.addParam(new RichDouble("Repulsion Mu", 0.35));
	//skeleton.addParam(new RichDouble("Repulsion Mu2", 0.15));
	//skeleton.addParam(new RichDouble("Follow Sample Radius", 0.33));
	//skeleton.addParam(new RichDouble("Follow Sample Max Angle", 80));// should add to UI
	//skeleton.addParam(new RichDouble("Inactive And Keep Virtual Angle", 60)); // should add to UI
	//skeleton.addParam(new RichDouble("Save Virtual Angle", 30)); // should add to UI
	average_power = 2.0;
	repulsion_power = 1.0;// "Repulsion Power"
	need_density=false;// Need Compute Density;
	radius = 0.001;// CGrid Radius;
	fix_original_weight=1.0;// "Fix Original Weight";
	h_gaussian_para = 4; //H Gaussian Para
	mu_max = 0.35;
	mu_min = 0.15;
	maxIterateTimes = 55;
	isNumSample = false;
	numofSamples =3000;
	percent = 0.1;
	TAG_STOP_RADIUS = 1.99;
	stopAndGrowError = 0.0005;

	iterate_time_in_one_stage = 0;
}


AxImprovedMeanshift::~AxImprovedMeanshift()
{
}

void AxImprovedMeanshift::initialRadius()
{
	if (!input_)
	{
		return;
	}
	if (input_->points.size() == 0)
		return;
	unsigned vn = input_->size();
	pcl::PointXYZ min_pt, max_pt;						// 沿参考坐标系坐标轴的边界值
	pcl::getMinMax3D(*input_, min_pt, max_pt);
	if (abs(min_pt.x - max_pt.x) < 1e-5 ||
		abs(min_pt.y - max_pt.y) < 1e-5 ||
		abs(min_pt.z - max_pt.z) < 1e-5)
	{
		double diagonal_length = sqrt((min_pt.getVector3fMap() - max_pt.getVector3fMap()).squaredNorm());
		double original_size = sqrt(double(vn));
		radius = 2 * init_para * diagonal_length / original_size;
	}
	else
	{
		double diagonal_length = sqrt((min_pt.getVector3fMap() - max_pt.getVector3fMap()).squaredNorm());
		double original_size = pow(double(vn), 0.333);
		radius = init_para * diagonal_length / original_size;
	}
}

void AxImprovedMeanshift :: run()
{
	if (!input_)
	{
		return;
	}
	if (input_->points.size() == 0)
		return;
	if (isNumSample)//固定数目采样
	{
		downSamplingFixedNum(numofSamples);
	}
	else
	{
		downSampling(percent);
	}
	
	initVertexes();
	while (true)
	{
		double iterate_error = wlopIterate();
		removeTooClosePoints();
		eigenThresholdIdentification();
		if (iterate_error < stopAndGrowError ||	iterate_time_in_one_stage > maxIterateTimes)
		{
			/*if (speed <= 0 || speed >= 100)
			{
				speed = 0.5;
			}
			radius *= (1 + speed);
			if (radius> TAG_STOP_RADIUS)*/
			{
				break;
			}
			
		}
	}
}

//降采样
void AxImprovedMeanshift::downSampling(float percent_)
{	
	srand(time(NULL));
	size_t selectedPointIndex = -1;
	size_t iter = 0;
	size_t numSlectedPts = floor(percent_*input_->size());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sub_samples = pcl_t_cloud;

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
		selectedPointIndex = nCard[iter]; 
		if (selectedPointIndex>0 && selectedPointIndex < numMax)
		{
			pcl::PointXYZ pt = input_->at(selectedPointIndex);
			sub_samples->push_back(pt);
		}
	} while (iter++ < numSlectedPts-1);
}

//降采样
void AxImprovedMeanshift::downSamplingFixedNum(int numSlectedPts_)
{
	srand(time(NULL));
	size_t selectedPointIndex = -1;
	size_t iter = 0;
	size_t numSlectedPts = numSlectedPts_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_t_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sub_samples = pcl_t_cloud;

	size_t numMax = input_->size();
	if (numofSamples>numMax)
	{
		numofSamples = numMax;
	}
	std::vector<size_t> nCard(numMax, 0);
	srand(time(NULL));
	for (int i = 0; i < numMax; i++)
	{
		nCard[i] = i;
	}
	std::random_shuffle(nCard.begin(), nCard.begin() + numMax);
	do
	{
		selectedPointIndex = nCard[iter];
		if (selectedPointIndex>0 && selectedPointIndex < numMax)
		{
			pcl::PointXYZ pt = input_->at(selectedPointIndex);
			sub_samples->push_back(pt);
		}
	} while (iter++ < numSlectedPts - 1);
}

void AxImprovedMeanshift::initVertexes()
{
	unsigned vn = sub_samples->size();
	for (unsigned i=0;i< sub_samples->size(); i++)
	{
		CVertex vi;
		const pcl::PointXYZ pt = sub_samples->at(i);
		const float & cx = pt.x;
		const float & cy = pt.y;
		const float & cz = pt.z;
		vi.mPoint = Eigen::Vector3f(cx, cy, cz);
		samples.push_back(vi);
	}

	repulsion.resize(vn, Eigen::Vector3f::Zero());
	average.resize(vn, Eigen::Vector3f::Zero());

	repulsion_weight_sum.resize(vn, 0);
	average_weight_sum.resize(vn, 0);
}

double AxImprovedMeanshift::wlopIterate()
{
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(sub_samples);

	for (long idx = 0; idx < sub_samples->size(); ++idx)
	{
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		const pcl::PointXYZ pt = sub_samples->at(idx);
		const float & cx = pt.x;
		const float & cy = pt.y;
		const float & cz = pt.z;
		if (tree->radiusSearch(pt, radius, nn_indices, nn_dists) == 0)//有错误，程序无法进入
		{
			continue;
		}

		CVertex& v = samples[idx];
		v.neighbors.clear();
		v.original_neighbors.clear();
		for (size_t k = 1; k < nn_indices.size(); k++)
		{
			int index_i = nn_indices[k];
			v.neighbors.push_back(index_i);
		}
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_original(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_original->setInputCloud(input_);
#pragma omp parallel 
	{
#pragma omp for
		for (long idx = 0; idx < sub_samples->size(); ++idx)
		{
			std::vector<int> nn_indices;
			std::vector<float> nn_dists;
			const pcl::PointXYZ pt = sub_samples->at(idx);
			const float & cx = pt.x;
			const float & cy = pt.y;
			const float & cz = pt.z;
			if (tree_original->radiusSearch(pt, radius, nn_indices, nn_dists) == 0)//有错误，程序无法进入
			{
				continue;
			}
			CVertex& v = samples[idx];
			for (size_t k = 1; k < nn_indices.size(); k++)
			{
				int index_i = nn_indices[k];
				v.original_neighbors.push_back(index_i);
			}
		}
	}
	//"Samples Initial");
	//GlobalFun::computeBallNeighbors(samples, NULL,para->getDouble("CGrid Radius"), samples->bbox);
	//GlobalFun::computeEigenWithTheta(samples, para->getDouble("CGrid Radius") / sqrt(para->getDouble("H Gaussian Para")));

	computeEigenWithTheta();
	//if (nTimeIterated == 0)
	//{
	//	//"Original Initial");
	//	GlobalFun::computeBallNeighbors(original, NULL,	para->getDouble("CGrid Radius"), original->bbox);

	//	original_density.assign(original->vn, 0);
	//	if (para->getBool("Need Compute Density"))
	//	{
	//		computeDensity(true, para->getDouble("CGrid Radius"));
	//	}
	//	time.end();
	//}

	////"Sample Original neighbor");
	//GlobalFun::computeBallNeighbors(samples, original,para->getDouble("CGrid Radius"), box);

	computeAverageTerm();

	computeRepulsionTerm();

	double min_sigma = std::numeric_limits<double>::max();
	double max_sigma = -1;
	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];
		if (v.eigen_confidence < min_sigma)
		{
			min_sigma = v.eigen_confidence;
		}
		if (v.eigen_confidence > max_sigma)
		{
			max_sigma = v.eigen_confidence;
		}
	}

	/*double mu_max = para->getDouble("Repulsion Mu");
	double mu_min = para->getDouble("Repulsion Mu2");*/
	double mu_length = abs(mu_max - mu_min);
	double sigma_length = abs(max_sigma - min_sigma);
	Eigen::Vector3f c;
	int moving_num = 0;
	double max_error = 0;

	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];
		if (v.is_fixed_sample || v.is_skel_ignore)
		{
			//continue;
		}
		c = v.mPoint;

		double mu = (mu_length / sigma_length) * (v.eigen_confidence - min_sigma) + mu_min;

		if (average_weight_sum[i] > 1e-20)
		{
			v.mPoint = average[i] / average_weight_sum[i];

		}
		if (repulsion_weight_sum[i] > 1e-20 && mu >= 0)
		{
			v.mPoint += repulsion[i] * (mu / repulsion_weight_sum[i]);
		}

		if (average_weight_sum[i] > 1e-20 && repulsion_weight_sum[i] > 1e-20)
		{
			moving_num++;
			Eigen::Vector3f diff = v.mPoint - c;
			double move_error = sqrt(diff.squaredNorm());

			error_x += move_error;
		}
	}
	error_x = error_x / moving_num;
	iterate_time_in_one_stage++;
	currentMoveError = error_x;
	return error_x;
}

void AxImprovedMeanshift::computeAverageTerm()
{
	/*double average_power = para->getDouble("Average Power");
	bool need_density = para->getBool("Need Compute Density");
	double radius = para->getDouble("CGrid Radius");
	double fix_original_weight = para->getDouble("Fix Original Weight");
	gaussian_para= para->getDouble("H Gaussian Para");*/
	double radius2 = radius * radius;
	double iradius16 = -h_gaussian_para / radius2;

	//cout << "Original Size:" << samples->vert[0].original_neighbors.size() << endl;
	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];

		if (v.is_fixed_sample) //Here is different from WLOP
		{
			average_weight_sum[i] = 0.;
			continue;
		}

		for (int j = 0; j < v.original_neighbors.size(); j++)
		{
			pcl::PointXYZ t = input_->at(v.original_neighbors[j]);
			Eigen::Vector3f t_mPoint(t.x, t.y, t.z);
			Eigen::Vector3f diff = v.mPoint - t_mPoint;
			double dist2 = diff.squaredNorm();

			double w = 1;
			if (average_power < 2)
			{
				double len = sqrt(dist2);
				if (len <= 0.001 * radius) len = radius*0.001;
				w = exp(dist2 * iradius16) / pow(len, 2 - average_power);
			}
			else
			{
				w = exp(dist2 * iradius16);
			}

			if (need_density)
			{
				//w *= original_density[t.m_index];
			}

			/*if (t.is_fixed_original)
			{
				w *= fix_original_weight;
			}*/

			average[i] += t_mPoint * w;
			average_weight_sum[i] += w;
		}
	}
}

void AxImprovedMeanshift::computeRepulsionTerm()
{
	//double repulsion_power = para->getDouble("Repulsion Power");
	//double radius = para->getDouble("CGrid Radius");

	double radius2 = radius * radius;
	double iradius16 = -h_gaussian_para / radius2;

	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];

		if (v.is_fixed_sample || v.is_skel_ignore)//Here is different from WLOP
		{
			repulsion_weight_sum[i] = 0.;
			continue;
		}

		for (int j = 0; j < v.neighbors.size(); j++)
		{
			CVertex& t = samples[v.neighbors[j]];
			Eigen::Vector3f diff = v.mPoint - t.mPoint;

			double dist2 = diff.squaredNorm();
			double len = sqrt(dist2);
			if (len <= 0.001 * radius) len = radius*0.001;

			double w = exp(dist2*iradius16);
			double rep = w * pow(1.0 / len, repulsion_power);

			repulsion[i] += diff * rep;
			repulsion_weight_sum[i] += rep;
		}
	}
}

void AxImprovedMeanshift::computeEigenWithTheta()
{
	double local_radius = radius/ sqrt(h_gaussian_para);
	std::vector<std::vector<int> > neighborMap;

	typedef std::vector<CVertex>::iterator VertexIterator;

	VertexIterator begin = samples.begin();
	VertexIterator end = samples.end();

	neighborMap.assign(end - begin, std::vector<int>());

	int curr_index = 0;

	for (VertexIterator iter = begin; iter != end; iter++, curr_index++)
	{
		if (iter->neighbors.size() <= 3)
		{
			iter->eigen_confidence = 0.5;
			continue;
		}

		for (int j = 0; j < iter->neighbors.size(); j++)
		{
			neighborMap[curr_index].push_back(iter->neighbors[j]);
		}
	}

	double radius2 = local_radius*local_radius;
	double iradius16 = -1 / radius2;

	int currIndex = 0;
	for (VertexIterator iter = begin; iter != end; iter++, currIndex++)
	{
		if (iter->neighbors.size() <= 3)
		{
			iter->eigen_confidence = 0.5;
			continue;
		}

		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector3f  diff;
		covariance_matrix.setZero();
		int neighborIndex = -1;
		int neighbor_size = iter->neighbors.size();
		for (unsigned int n = 0; n < neighbor_size; n++)
		{
			neighborIndex = neighborMap[currIndex][n];
			if (neighborIndex < 0)
				break;
			VertexIterator neighborIter = begin + neighborIndex;

			diff = iter->mPoint - neighborIter->mPoint;

			Eigen::Vector3f vm = iter->eigen_vector2_normal;
			Eigen::Vector3f tm = neighborIter->eigen_vector2_normal;
			double dist2 = diff.squaredNorm();
			double theta = exp(dist2 * iradius16);

			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
					covariance_matrix(i, j) += diff[i] * diff[j] * theta;
		}

		Eigen::Vector3f eigenvalues;
		Eigen::Matrix3f	eigenvectors;
		pcl::eigen33(covariance_matrix, eigenvectors, eigenvalues);
		double sum_eigen_value = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
		iter->eigen_confidence = eigenvalues[0] / sum_eigen_value;

		for (int d = 0; d < 3; d++)
			iter->eigen_vector0[d] = eigenvectors(d, 0);
		for (int d = 0; d < 3; d++)
			iter->eigen_vector1[d] = eigenvectors(d, 1);
		for (int d = 0; d < 3; d++)
			iter->eigen_vector2_normal[d] = eigenvectors(d, 2);

		iter->eigen_vector0.normalize();
		iter->eigen_vector1.normalize();
		iter->eigen_vector2_normal.normalize();
	}
}

void AxImprovedMeanshift::removeTooClosePoints()
{
	double near_threshold = 0.0100;// para->getDouble("Combine Too Close Threshold");
	double near_threshold2 = near_threshold * near_threshold;

	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];
		for (int j = 0; j < v.neighbors.size(); j++)
		{
			CVertex& t = samples[v.neighbors[j]];

			if (!t.isSample_JustMoving())
			{
				continue;
			}

			double dist2 = (v.mPoint-t.mPoint).squaredNorm();
			if (dist2 < near_threshold2)
			{
				t.remove();
			}
		}
	}
}

void AxImprovedMeanshift::eigenThresholdIdentification()
{
	int sigma_KNN = 6;// para->getDouble("Sigma KNN");
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_wlopOnce(new pcl::PointCloud<pcl::PointXYZ>);
	//GlobalFun::computeAnnNeigbhors(samples->vert, samples->vert, sigma_KNN, false, "void Skeletonization::eigenThresholdClassification()");
	for (int i=0;i<samples.size(); ++i)
	{
		CVertex vi = samples[i];
		pcl::PointXYZ pt(vi.mPoint.x(), vi.mPoint.y(),vi.mPoint.z());
		pcl_cloud_wlopOnce->push_back(pt);
	}
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(pcl_cloud_wlopOnce);

	for (long idx = 0; idx < pcl_cloud_wlopOnce->size(); ++idx)
	{
		std::vector<int> nn_indices;
		std::vector<float> nn_dists;
		const pcl::PointXYZ pt = pcl_cloud_wlopOnce->at(idx);
		if (tree->radiusSearch(pt, radius, nn_indices, nn_dists) == 0)//有错误，程序无法进入
		{
			continue;
		}

		CVertex& v = samples[idx];
		v.neighbors.clear();
		v.original_neighbors.clear();
		for (size_t k = 1; k < nn_indices.size(); k++)
		{
			int index_i = nn_indices[k];
			v.neighbors.push_back(index_i);
		}
	}
	/*if (para->getBool("Use Compute Eigen Ignore Branch Strategy"))
	{
		GlobalFun::computeEigenIgnoreBranchedPoints(samples);
	}
	else*/
	{
		computeEigen(samples);
	}

	eigenConfidenceSmoothing();

	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];

		if (!v.isSample_Moving())
		{
			continue;
		}

		double eigen_psi = v.eigen_confidence;
		double eigen_threshold = 0.901;// para->getDouble("Eigen Feature Identification Threshold");

		if (eigen_psi > eigen_threshold)
		{
			v.is_fixed_sample = true;
		}
		else
		{
			v.is_fixed_sample = false;
		}
	}
}

void AxImprovedMeanshift::computeEigen(std::vector<CVertex>& samples_)
{
	std::vector<std::vector<int> > neighborMap;

	typedef std::vector<CVertex>::iterator VertexIterator;

	VertexIterator begin = samples_.begin();
	VertexIterator end = samples_.end();

	neighborMap.assign(end - begin, std::vector<int>());

	int curr_index = 0;

	int currIndex = 0;
	for (VertexIterator iter = begin; iter != end; iter++, currIndex++)
	{
		Eigen::Matrix3f covariance_matrix;
		Eigen::Vector3f  diff;
		covariance_matrix.setZero();
		int neighbor_size = iter->neighbors.size();
		for (unsigned int n = 0; n<neighbor_size; n++)
		{
			Eigen::Vector3f& tP = samples_[iter->neighbors[n]].mPoint;
			diff = iter->mPoint - tP;

			for (int i = 0; i<3; i++)
				for (int j = 0; j<3; j++)
					covariance_matrix(i, j) += diff[i] * diff[j];
		}


		Eigen::Vector3f   eigenvalues;
		Eigen::Matrix3f	eigenvectors;
		int required_rotations;
		pcl::eigen33(covariance_matrix, eigenvectors, eigenvalues);

		double sum_eigen_value = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);

		iter->eigen_confidence = eigenvalues[0] / sum_eigen_value;
		for (int d = 0; d<3; d++)
			iter->eigen_vector0[d] = eigenvectors(d, 0);
		for (int d = 0; d<3; d++)
			iter->eigen_vector1[d] = eigenvectors(d, 1);
		for (int d = 0; d<3; d++)
			iter->eigen_vector2_normal[d] = eigenvectors(d, 2);

		iter->eigen_vector0.normalize();
		iter->eigen_vector1.normalize();
		iter->eigen_vector2_normal.normalize();
	}

}

void AxImprovedMeanshift::eigenConfidenceSmoothing()
{
	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];
		v.eigen_confidence = 1 - v.eigen_confidence;
	}

	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];
		double sum = v.eigen_confidence;
		for (int j = 0; j < v.neighbors.size(); j++)
		{
			sum += samples[v.neighbors[j]].eigen_confidence;
		}
		v.eigen_confidence = sum / (v.neighbors.size() + 1);
	}

	for (int i = 0; i < samples.size(); i++)
	{
		CVertex& v = samples[i];
		v.eigen_confidence = 1 - v.eigen_confidence;

		if (v.eigen_confidence < 0)
		{
			v.eigen_confidence = 0.5;
		}
	}
}

std::vector<CVertex> AxImprovedMeanshift::getResults()
{
	return samples;
}

void AxImprovedMeanshift::setInputPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_0)
{
	input_ = input_0;
}

void AxImprovedMeanshift::setNumSamplesParameter(bool isNum_, float percent_, unsigned num_)
{
	isNumSample = isNum_;
	numofSamples = num_;
	percent = percent_;
}