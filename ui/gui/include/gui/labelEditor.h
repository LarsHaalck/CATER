#ifndef GUI_LABEL_EDITOR_H
#define GUI_LABEL_EDITOR_HH

#include <QDialog>
#include <QStandardItemModel>
#include <QItemSelectionModel>
#include <set>

#include "gui/colorDelegate.h"
#include "gui/keyDelegate.h"

namespace Ui
{
class LabelEditor;
}

namespace gui
{
class LabelEditor : public QDialog
{
    Q_OBJECT

public:
    explicit LabelEditor(QWidget* parent = nullptr);

    ~LabelEditor();
private:
    QList<QStandardItem*> getDefaultItems() const;
    void blockResetItem(int row);
    void unblockItem(int row);
    bool isBlocked(int row);
    bool checkDuplicate() const;

private slots:
    void on_buttonNewGroup_clicked();
    void on_buttonNewLabel_clicked();
    void on_buttonDelete_clicked();

    void on_accepted();

private:
    Ui::LabelEditor* ui;
    QStandardItemModel mModel;
    QItemSelectionModel mSelectionModel;

    ColorDelegate mColorDelegate;
    KeyDelegate mKeyDelegate;
};
} // namespace gui
#endif // GUI_LABEL_EDITOR_H
