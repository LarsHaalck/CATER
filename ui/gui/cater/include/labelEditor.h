#ifndef GUI_LABEL_EDITOR_H
#define GUI_LABEL_EDITOR_H

#include <QDialog>
#include <QItemSelectionModel>
#include <QStandardItemModel>
#include <set>

#include "colorDelegate.h"
#include "keyDelegate.h"
#include "labelConfig.h"

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
    void setLabelConfigs(const LabelGroupConfigs& config);
    LabelGroupConfigs getLabelConfigs() const;

private:
    QList<QStandardItem*> getDefaultItems(bool blocked) const;
    QList<QStringList> getElements(int column) const;
    bool checkDuplicate(int column, int level = -1) const;
    bool checkEmpty(int column) const;
    bool validate();

    int keyStringToInt(const QString& key) const;
    QString keyIntToString(int key) const;

private slots:
    void on_buttonNewGroup_clicked();
    void on_buttonNewLabel_clicked();
    void on_buttonReset_clicked();
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
