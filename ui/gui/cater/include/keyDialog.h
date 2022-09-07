#ifndef GUI_KeyDialog_H
#define GUI_KeyDialog_H

#include <QDialog>

namespace Ui
{
class KeyDialog;
}

namespace gui
{
class KeyDialog : public QDialog
{
    Q_OBJECT

public:
    explicit KeyDialog(QWidget* parent = nullptr);
    ~KeyDialog();

    void keyPressEvent(QKeyEvent* event);

    int getKey() const { return mKey; }
    QString getKeyName() const { return mKeyName; }

    void setKey(int key);
    void setKeyName(QString key);

private:
    Ui::KeyDialog* ui;
    int mKey;
    QString mKeyName;
};
} // namespace gui
#endif // GUI_KEY_DIALOG_H
