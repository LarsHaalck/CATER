#include "gui/keyDialog.h"
#include "ui_keyDialog.h"

#include <QMessageBox>
#include <QKeyEvent>
#include <QPushButton>

namespace gui
{
KeyDialog::KeyDialog(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::KeyDialog)
    , mKey(-1)
    , mKeyName()
{
    ui->setupUi(this);
    ui->key->setText("");
    ui->buttonBox->button(QDialogButtonBox::Ok)->setDefault(true);
}

KeyDialog::~KeyDialog()
{
    this->blockSignals(true);
    delete ui;
}


void KeyDialog::setKey(int key)
{
    mKey = key;
    ui->key->setText(QKeySequence(key).toString());
}

void KeyDialog::setKeyName(QString key)
{
    mKey = QKeySequence::fromString(key)[0];
    ui->key->setText(key);
}


void KeyDialog::keyPressEvent(QKeyEvent* event)
{
    if (event->modifiers() != Qt::NoModifier)
    {
        QMessageBox::information(this, "Info", "Please don't use modifier keys");
        return;
    }

    if (event->key() == Qt::Key_Return)
        QDialog::keyPressEvent(event);

    mKey = event->key();
    mKeyName = event->text();
    ui->key->setText(mKeyName);
}

} // namespace gui
