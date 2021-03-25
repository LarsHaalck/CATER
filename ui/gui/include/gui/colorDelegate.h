#ifndef GUI_COLOR_DELEGATE_H
#define GUI_COLOR_DELEGATE_H

#include <QItemDelegate>

namespace gui
{
class ColorDelegate : public QItemDelegate
{
    Q_OBJECT
public:
    explicit ColorDelegate(QObject* parent = 0);

    QWidget* createEditor(
        QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const;
    void setEditorData(QWidget* editor, const QModelIndex& index) const;
    void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const;
    void paint(
        QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const;
};
} // namespace gui

#endif // GUI_COLOR_DELEGATE_H
