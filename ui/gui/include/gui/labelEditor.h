#ifndef GUI_LABEL_EDITOR_H
#define GUI_LABEL_EDITOR_HH

#include <QDialog>
#include <QStandardItemModel>
#include <QItemSelectionModel>
#include <set>

#include "gui/colorDelegate.h"
#include "gui/keyDelegate.h"
#include "gui/labelConfig.h"

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
    LabelGroupConfigs getLabelConfigs() const;

private:
    QList<QStandardItem*> getDefaultItems() const;
    void blockResetItem(int row);
    void unblockItem(int row);
    bool isBlocked(int row);
    QList<QStringList> getElements(int column) const;
    bool checkDuplicate(int column, int level = -1) const;
    bool checkEmpty(int column) const;
    bool validate();

    int keyStringToInt(const QString& key) const;

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
