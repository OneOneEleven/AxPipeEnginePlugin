#pragma once

#include "axpipeengineplugin_global.h"
#ifndef AXPIPEENGINEPLUGIN_HEADER
#define AXPIPEENGINEPLUGIN_HEADER

//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: AxCityEnginePlugin                 #
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
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

#include "ccStdPluginInterface.h"
#include <QtGui>
#include <ppl.h>
#include <ccPointCloud.h>
#include <ccPlane.h>
#include <ccPolyline.h>
#include <ccGBLSensor.h>
#include <ccHObjectCaster.h>
#include <ScalarField.h>
#include "AxPipeLineTracking.h"
#include "AxImprovedMeanshift.h"
#include "UI/ccDlgPipelineDetector.h"
#include "UI/ccDlgImprovedMeanshift.h"

class AXPIPEENGINEPLUGIN_EXPORT AxPipeEnginePlugin : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccStdPluginInterface)

	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.AxPipeEnginePlugin" FILE "info.json")

public:
	explicit AxPipeEnginePlugin(QObject *parent = nullptr);
	~AxPipeEnginePlugin() override = default;

	// inherited from ccStdPluginInterface
	void onNewSelection(const ccHObject::Container &selectedEntities) override;
	QList<QAction *> getActions() override;

private:

	void doAction();
	//管道提取
	void do3DPipelineDetection();
	//管道提取
	void doImprovedMeanshift();


	QAction* m_action;
	QAction* m_action_3DPipeline;
	QAction* m_action_WLOP;
public:

};

#endif