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

#include "ccDlgImprovedMeanshift.h"
#include "qstandarditemmodel.h"


static double s_inputResolution = 0.025;	//resolution in mm


ccDlgImprovedMeanshift::ccDlgImprovedMeanshift(QWidget* parent)
	: QDialog(parent, Qt::Tool)
	, Ui::DlgImprovedMeanshift()
{
	setupUi(this);


}

void ccDlgImprovedMeanshift::saveSettings()
{

}
