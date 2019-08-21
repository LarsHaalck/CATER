#include "gui/mainWindow.h"
#include "ui_mainWindow.h"

namespace gui
{
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow() { delete ui; }
}
