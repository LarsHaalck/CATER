#ifndef PANOWIZARD_H
#define PANOWIZARD_H

#include <QMainWindow>

namespace Ui {
class PanoWizard;
}

namespace gui
{

class PanoWizard : public QMainWindow
{
    Q_OBJECT

public:
    explicit PanoWizard(QWidget *parent = nullptr);
    ~PanoWizard();

private:
    Ui::PanoWizard *ui;
};

} // namespace gui

#endif // PANOWIZARD_H
