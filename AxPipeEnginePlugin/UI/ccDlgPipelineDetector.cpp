//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qNDTRANSAC_SD                    #
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
//#               COPYRIGHT:Fan Yang                      #
//#                                                                        #
//##########################################################################

#include "ccDlgPipelineDetector.h"
#include "qstandarditemmodel.h"


static double s_inputResolution = 0.025;	//resolution in mm


ccDlgPipelineDetector::ccDlgPipelineDetector(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::DlgPipelineDetector()
{
	setupUi(this);
	inputTolerance->setValue(s_inputResolution);

}

void ccDlgPipelineDetector::saveSettings()
{
	s_inputResolution = inputTolerance->value();
}
void ccDlgPipelineDetector::ChooseFilePath()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open ply list file"), " ", tr("ply list(*.list);;Allfile(*.*)"));
	if (!fileName.isEmpty())
	{
		//txtPath->setText(fileName);
	}
}
void ccDlgPipelineDetector::ChooseFilePath2()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("open ply list file"), " ", tr("ply list(*.list);;Allfile(*.*)"));
	if (!fileName.isEmpty())
	{
		//txtPath_2->setText(fileName);
	}
}