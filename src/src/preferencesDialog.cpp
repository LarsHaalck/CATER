#include "gui/preferencesDialog.h"
#include "ui_preferencesDialog.h"

namespace gui
{
PreferencesDialog::PreferencesDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreferencesDialog)
{
    ui->setupUi(this);
}

PreferencesDialog::~PreferencesDialog()
{
    delete ui;
}
} // namespace gui
