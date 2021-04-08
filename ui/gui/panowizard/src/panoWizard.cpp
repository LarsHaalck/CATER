#include "panoWizard.h"
#include "ui_panoWizard.h"

namespace gui
{
PanoWizard::PanoWizard(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PanoWizard)
{
    ui->setupUi(this);
}

PanoWizard::~PanoWizard()
{
    delete ui;
}
} // namespace gui
