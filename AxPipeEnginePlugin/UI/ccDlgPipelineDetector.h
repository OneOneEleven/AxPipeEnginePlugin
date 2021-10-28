#ifndef CC_DLG_PIPELINEETECTOR_HEADER
#define CC_DLG_PIPELINEETECTOR_HEADER

#include "ui_dlgPipelineDetector.h"
#include <QFileDialog>


class ccDlgPipelineDetector : public QDialog, public Ui::DlgPipelineDetector
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccDlgPipelineDetector(QWidget* parent = 0);

protected slots:

	//! Saves (temporarily) the dialog paramters on acceptation
	void saveSettings();
	void ChooseFilePath();
	void ChooseFilePath2();

};

#endif
