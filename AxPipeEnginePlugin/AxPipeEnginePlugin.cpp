#include "AxPipeEnginePlugin.h"

AxPipeEnginePlugin::AxPipeEnginePlugin(QObject *parent)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/qPipeEnginePlugin/info.json")
	, m_action(nullptr)
	, m_action_3DPipeline(0)
	, m_action_WLOP(0)
{
}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void AxPipeEnginePlugin::onNewSelection(const ccHObject::Container &selectedEntities)
{
	if (m_action == nullptr)
	{
		return;
	}

	// If you need to check for a specific type of object, you can use the methods
	// in ccHObjectCaster.h or loop and check the objects' classIDs like this:
	//
	//	for ( ccHObject *object : selectedEntities )
	//	{
	//		if ( object->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
	//		{
	//			// ... do something with the viewports
	//		}
	//	}

	// For example - only enable our action if something is selected.
	m_action->setEnabled(!selectedEntities.empty());
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> AxPipeEnginePlugin::getActions()
{
	if (!m_action)
	{
		// Here we use the default plugin name, description, and icon,
		// but each action should have its own.
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(QIcon(":/CC/qPipeEnginePlugin/images/icon.png"));

		// Connect appropriate signal
		connect(m_action, &QAction::triggered, this, &AxPipeEnginePlugin::doAction);
	}

	if (!m_action_3DPipeline)//3D�ܵ���ȡ
	{
		m_action_3DPipeline = new QAction("Detect3DPipeline", this);
		m_action_3DPipeline->setToolTip("3D Pipeline Detection");
		m_action_3DPipeline->setIcon(QIcon(":/CC/qPipeEnginePlugin/images/qPipe50.png"));
		//connect appropriate signal
		connect(m_action_3DPipeline, &QAction::triggered, this, &AxPipeEnginePlugin::do3DPipelineDetection);
	}

	if (!m_action_WLOP)//3D�ܵ���ȡ
	{
		m_action_WLOP = new QAction("ImprovedMeanshift", this);
		m_action_WLOP->setToolTip("Improved Mean shift");
		m_action_WLOP->setIcon(QIcon(":/CC/qPipeEnginePlugin/images/qCentre.png"));
		//connect appropriate signal
		connect(m_action_WLOP, &QAction::triggered, this, &AxPipeEnginePlugin::doImprovedMeanshift);
	}

	return{ m_action,m_action_3DPipeline,m_action_WLOP };
}

//�ܵ��������ȡ
void AxPipeEnginePlugin::do3DPipelineDetection()
{
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Please select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccPointCloud* theCloud = static_cast<ccPointCloud*>(ent);
	if (!theCloud)
		return;
	unsigned count = theCloud->size();
	bool hasNorms = theCloud->hasNormals();
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_(new pcl::PointCloud<pcl::PointXYZ>);
	CC2PCL_PointCloud(*theCloud, *input_);	//ת�����Ƹ�ʽ
	ccDlgPipelineDetector lddlg;
	if (input_->points.size() > 3)
	{
		double scale = 0.0;
		std::vector<double> ptScales;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(input_);
		for (size_t i = 0; i < input_->points.size(); i++)
		{
			double x = input_->points.at(i).x;
			double y = input_->points.at(i).y;
			std::vector<int> nn_indices;
			std::vector<float> nn_dists;
			const pcl::PointXYZ pt = input_->at(i);
			if (tree->nearestKSearch(pt, 5, nn_indices, nn_dists) == 0)//�д��󣬳����޷�����
			{
				continue;
			}
			double dist = sqrt(nn_dists[1]);//sqrt(nn_dists[3]);Ϊɶ��3����
			ptScales.push_back(dist);
			scale += dist;
		}

		std::sort(ptScales.begin(), ptScales.end(), [](const double& lhs, const double& rhs) { return lhs < rhs; });
		int idxNinety = std::min(int(double(ptScales.size()) * 0.9), int(ptScales.size() - 1));
		double gridSideLength = ptScales[idxNinety] * 0.75 *  2;
		scale = scale / input_->points.size();
		double thLineLength = 10 * scale;//thLineLength = 8 * scale;
		lddlg.inputRmin->setValue(thLineLength);
		lddlg.inputRmax->setValue(thLineLength * 1.5 * 1.5);
		lddlg.inputTolerance->setValue(gridSideLength);
		double gridSideLength2 = ptScales[idxNinety] * 0.75 * 3;
		lddlg.inputThickness->setValue(gridSideLength2);
		double gridSideLength3 = ptScales[idxNinety] * 3;
		lddlg.inputRmax_Sysmmetry->setValue(gridSideLength3);
	}	
	if (!lddlg.exec())
		return;
	//�������룺��������뾶������С�����뾶��---------------------------
	double r_max = lddlg.inputRmax->value();
	double r_min = lddlg.inputRmin->value();
	int iter_num = lddlg.sbIteration->value();
	double threshold_dist = lddlg.inputTolerance->value();
	double thickness = lddlg.inputThickness->value();
	float percent = lddlg.horizontalSlider->value();
	int sys_iter_num = lddlg.sbIteration_Sysmmetry->value();//�ԳƼ���������
	float sys_max_corresp_reflected_distance = lddlg.inputRmax_Sysmmetry->value();
	float sym_min_corresp_distance = sys_max_corresp_reflected_distance * 4;
	float max_sym_normal_fit_error = (pcl::deg2rad(45.0f));
	float min_angle = lddlg.inputMinAngle->value();
	float max_angle = lddlg.inputMaxAngle->value();
	QElapsedTimer eTimer;
	eTimer.start();
	//�ܵ���ȡ-----------------------------------------------------------
	AxPipeLineTracking track;
	track.reject_min_angle = min_angle;
	track.reject_max_angle = max_angle;//�Ƕ�
	track.setInputPointCloud(input_);
	track.downSampling(percent / 100);//���Ȳ������ò�����Ps
	track.setParametersRaduis(r_max, r_min);
	track.setParameters(iter_num, threshold_dist, thickness);
	track.setParametersSysmmetry(sys_iter_num, max_sym_normal_fit_error, sym_min_corresp_distance, sys_max_corresp_reflected_distance);
	//track.compute();
	track.computeEx();	//�뾶R1�����ݻ��潫��ǰ������������Բ��
	//��ʾ���------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZI>::Ptr output_ = track.getFinalSamples();
	int numCircleCenters = output_->size();
	ccHObject* group = 0;
	if (!group)
		group = new ccHObject(QString("PipeLineCenters").arg(ent->getName()));
	ccPointCloud *ccCloud = new ccPointCloud();
	ccCloud->reserve(numCircleCenters);
	for (int j = 0; j < numCircleCenters; ++j)
	{
		float x = output_->at(j).x;
		float y = output_->at(j).y;
		float z = output_->at(j).z;
		CCVector3 cvc(x, y, z);
		ccCloud->addPoint(cvc);
	}
	QString sfName = "Radius";
	int sfIdx = -1;
	sfIdx = ccCloud->getScalarFieldIndexByName(qPrintable(sfName));
	if (sfIdx < 0)
		sfIdx = ccCloud->addScalarField(qPrintable(sfName));
	if (sfIdx >= 0)
		ccCloud->setCurrentInScalarField(sfIdx);
	else
	{
		m_app->dispToConsole(QString("Failed to create scalar field on cloud '%1' (not enough memory?)").arg(ccCloud->getName()));
	}
	for (int j = 0; j < numCircleCenters; ++j)
	{
		float iniensity_1 = output_->at(j).intensity;
		ccCloud->setPointScalarValue(j, iniensity_1);
	}
	if (ccCloud && sfIdx >= 0)
	{
		//���õ�ǰ��ʾ��ScalarField
		ccCloud->setCurrentDisplayedScalarField(sfIdx);
		ccCloud->showSF(sfIdx >= 0);
		ccCloud->getCurrentInScalarField()->computeMinAndMax();
	}
	theCloud->prepareDisplayForRefresh();
	ccCloud->setName(QString("PipeLineCenters"));
	ccColor::Rgb col = ccColor::Generator::Random();
	ccCloud->setRGBColor(col);
	ccCloud->showColors(true);
	ccCloud->setPointSize(1);
	group->addChild(ccCloud);
	group->setVisible(true);
	m_app->addToDB(group);
	qint64 elapsedTime_ms = eTimer.elapsed();
	float timell = static_cast<float>(elapsedTime_ms) / 1.0e3;
	QString infomation = QString("PipeLine Centers Extracted Done! Total Time Cost: {%1s}").arg(timell);
	m_app->dispToConsole(infomation, ccMainAppInterface::STD_CONSOLE_MESSAGE);
}

void AxPipeEnginePlugin::doImprovedMeanshift()
{
	if (!m_app)
		return;

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Please select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccDlgImprovedMeanshift lddlg;
	if (!lddlg.exec())
		return;
	//�������룺��������뾶������С�����뾶��---------------------------
	bool checkedNum = lddlg.radioButton_2->isChecked();
	float percent = lddlg.horizontalSlider->value();
	unsigned count = lddlg.spinBox_SamplePoints->value();
	ccPointCloud* theCloud = static_cast<ccPointCloud*>(ent);
	if (!theCloud)
		return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_(new pcl::PointCloud<pcl::PointXYZ>);
	CC2PCL_PointCloud(*theCloud, *input_);	//ת�����Ƹ�ʽ
	AxImprovedMeanshift ims;
	ims.setInputPointCloud(input_);
	ims.setNumSamplesParameter(checkedNum, percent, count);
	QElapsedTimer eTimer;
	eTimer.start();
	ims.initialRadius();
	ims.run();
	qint64 elapsedTime_ms = eTimer.elapsed();

	std::vector<CVertex> output_ = ims.getResults();
	int numCircleCenters = output_.size();
	ccHObject* group = 0;
	if (!group)
		group = new ccHObject(QString("Centers").arg(ent->getName()));
	ccPointCloud *ccCloud = new ccPointCloud();
	ccCloud->reserve(numCircleCenters);
	for (int j = 0; j < numCircleCenters; ++j)
	{
		float x = output_[j].mPoint.x();
		float y = output_[j].mPoint.y();
		float z = output_[j].mPoint.z();
		CCVector3 cvc(x, y, z);
		ccCloud->addPoint(cvc);
	}
	theCloud->prepareDisplayForRefresh();
	ccCloud->setName(QString("Centers"));
	ccColor::Rgb col = ccColor::Generator::Random();
	ccCloud->setRGBColor(col);
	ccCloud->showColors(true);
	ccCloud->setPointSize(1);
	group->addChild(ccCloud);
	group->setVisible(true);
	m_app->addToDB(group);
	float timell = static_cast<float>(elapsedTime_ms) / 1.0e3;
	QString infomation = QString("Improved meanshift done! Total Time Cost: {%1s}").arg(timell);
	m_app->dispToConsole(infomation, ccMainAppInterface::STD_CONSOLE_MESSAGE);

}

void AxPipeEnginePlugin::doAction()
{
	if (m_app == nullptr)
	{
		Q_ASSERT(false);
		return;
	}
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	size_t selNum = selectedEntities.size();
	if (selNum != 1)
	{
		m_app->dispToConsole("Please select one cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccHObject* ent = selectedEntities[0];
	assert(ent);
	if (!ent || !ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Select a real point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	ccPointCloud* theCloud = static_cast<ccPointCloud*>(ent);
	if (!theCloud)
		return;
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_(new pcl::PointCloud<pcl::PointXYZ>);
	CC2PCL_PointCloud(*theCloud, *input_);	//ת�����Ƹ�ʽ
	if (!input_)
	{
		return;
	}
	if (input_->points.size() == 0)
		return;
	unsigned vn = input_->size();
	pcl::PointXYZ min_pt, max_pt;						// �زο�����ϵ������ı߽�ֵ
	pcl::getMinMax3D(*input_, min_pt, max_pt);
	//------------------------------------------------------------------------------------------------
	double radius = 0;
	double init_para = 1.0;
	if (abs(min_pt.x - max_pt.x) < 1e-5 || abs(min_pt.y - max_pt.y) < 1e-5 || abs(min_pt.z - max_pt.z) < 1e-5)
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
	QString infomation = QString("Recommended radius: {%1}").arg(radius);
	m_app->dispToConsole(infomation, ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//---------------------------------------------------------------------------------------------------------
	double diff_dx = max_pt.x - min_pt.x ;
	double diff_dy = min_pt.y - max_pt.y;
	double diff_dz = min_pt.z - max_pt.z;
	double scale = std::max(std::max(diff_dx, diff_dy), diff_dz);
	//����������뾶��С
	double bitmapEpsilonDouble = .01f * scale;// set bitmap resolution (= sampling resolution) to 1% of bounding box width
	double radius2=.02f * scale;//ע��˴����ü��㷨�����Ĳ���
	double maxDistancetoPlane =.005f * scale;
	double epsilonDoubleSpinBox = .001f * scale;
	QString infomation2 = QString("Recommended radius2: {%1}/{%2}").arg(bitmapEpsilonDouble).arg(maxDistancetoPlane);
	m_app->dispToConsole(infomation2, ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//-----------------------------------------------------------------------------------------------------------
	std::vector<double> distList;
	float res = 0.0;//����ƽ������
	float var = 0.0;//���巽��
	float standard_deviation = 0.0;
	int n_points = 0;//�����¼��������
	int nres;//���������������
			 //vector��˳��������һ�֡�vector �ǿɱ䳤�Ķ�̬����
	std::vector<int> indices(2);//����һ������2��int�������ݵ�vector //����һ����̬���飬�洢��ѯ��������� //�ȼ��������д��� using std::vector; vector<int> indices(2);
	std::vector<float> sqr_distances(2);//�洢���ڵ��Ӧƽ������
	pcl::KdTreeFLANN<pcl::PointXYZ> tree;//��k-d tree��ʽ����
	tree.setInputCloud(input_);
	for (size_t i = 0; i < input_->size(); ++i)//ѭ������ÿһ����
	{
		if (!pcl_isfinite(input_->points[i].x))//pcl_isfinite��������һ������ֵ�����ĳ��ֵ�ǲ���������ֵ
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		// kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) 
		//����ִ�� K ���ڲ��ҵĳ�Ա���������У���kΪ1��ʱ�򣬾����������������k����1��ʱ�򣬾��Ƕ��������������˴�kΪ2��
		//KΪҪ�������ھ�������k the number of neighbors to search for��
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);//��������ֵ�������ҵ���������������return number of neighbors found
		if (nres == 2)//���Ϊ������֮��
		{
			double dist= sqrt(sqr_distances[1]);//sqrt()����������sqr_distances[1]�Ŀ�ƽ����
			res += dist;
			//std::cout << "sqr_distances[1]��" << sqr_distances[1] << std::endl;//��ӡ���ٽ�������ƽ��ֵ
			++n_points;
			distList.push_back(dist);
		}
	}
	std::cout << "nres��" << nres << std::endl;
	std::cout << "����������n_points��" << n_points << std::endl;
	if (n_points != 0)
	{
		res /= n_points;
		for (size_t i = 0; i < input_->size(); ++i)
		{
			if (!pcl_isfinite(input_->points[i].x))
			{
				continue;
			}
			nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
			if (nres == 2)
			{
				var += pow(sqrt(sqr_distances[1]) - res, 2);
				++n_points;
			}
		}
		if (n_points != 0)
		{
			var /= n_points;
			standard_deviation = sqrt(var);
		}
	}
	std::cout << "ƽ�����룺" << res << std::endl;
	std::cout << "���" << var << std::endl;
	std::cout << "��׼�" << standard_deviation << std::endl;
	Concurrency::parallel_sort(distList.begin(), distList.end(),[](const double& lhs, const double& rhs) { return lhs < rhs; });
	double maxDist = distList[0];
	double minDist = distList[distList.size()-1];
	int mid = floor(distList.size() / 2);
	double midDist = distList[mid];
	QString infomation3 = QString("Mean {%1}; var {%2}; standard_deviation {%3}").arg(res).arg(var).arg(standard_deviation);
	m_app->dispToConsole(infomation3, ccMainAppInterface::STD_CONSOLE_MESSAGE);
	QString infomation4 = QString("Max {%1}; Mid {%2}; Min {%3}").arg(maxDist).arg( midDist).arg(minDist);
	m_app->dispToConsole(infomation4, ccMainAppInterface::STD_CONSOLE_MESSAGE);
	//--------------------------------------------------------------------------------------------
	int idxNinety = std::min(int(double(distList.size()) * 0.9), int(distList.size() - 1));
	double gridSideLength = distList[idxNinety] * 0.75;
	double thLineLength = 2 * res;
	QString infomation5 = QString("Dist1{%1}; Dist1 {%2};").arg(gridSideLength).arg(thLineLength);
	m_app->dispToConsole(infomation5, ccMainAppInterface::STD_CONSOLE_MESSAGE);
	/*m_app->dispToConsole("[AxPipeEnginePlugin] Warning: example plugin shouldn't be used as is", ccMainAppInterface::WRN_CONSOLE_MESSAGE);
	m_app->dispToConsole("AxPipeEnginePlugin plugin shouldn't be used - it doesn't do anything!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);*/
}
