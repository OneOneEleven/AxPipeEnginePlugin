#ifndef CC_DLG_IMPROVEDMEANSHIFT_HEADER
#define CC_DLG_IMPROVEDMEANSHIFT_HEADER

#include "ui_dlgImprovedMeanshift.h"
#include <QFileDialog>


class ccDlgImprovedMeanshift : public QDialog, public Ui::DlgImprovedMeanshift
{
	Q_OBJECT

public:

	//! Default constructor
	explicit ccDlgImprovedMeanshift(QWidget* parent = 0);

protected slots:

	//! Saves (temporarily) the dialog paramters on acceptation
	void saveSettings();


};

#endif
