#ifndef GUI_KEY_DELEGATE_H
#define GUI_KEY_DELEGATE_H

#include <QItemDelegate>

namespace gui
{
class KeyDelegate : public QItemDelegate
{
    Q_OBJECT
public:
    explicit KeyDelegate(QObject* parent = 0);
    QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const;
    void setEditorData(QWidget* editor, const QModelIndex& index) const;
    void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const;
};
} // namespace gui

#endif // GUI_KEY_DELEGATE_H
