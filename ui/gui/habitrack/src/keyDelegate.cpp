#include "keyDelegate.h"

#include "keyDialog.h"

#include <QPainter>
#include <iostream>

namespace gui
{
KeyDelegate::KeyDelegate(QObject* parent)
    : QItemDelegate(parent)
{
}

QWidget* KeyDelegate::createEditor(
    QWidget* parent, const QStyleOptionViewItem&, const QModelIndex&) const
{
    auto editor = new KeyDialog(parent);
    editor->setModal(true);
    return editor;
}

void KeyDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    auto ptr = index.model()->data(index, Qt::EditRole);
    auto key = index.model()->data(index, Qt::EditRole).toString();
    auto keyDialog = static_cast<KeyDialog*>(editor);

    if (key.size())
        keyDialog->setKeyName(key);
}

void KeyDelegate::setModelData(
    QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    auto keyDialog = static_cast<KeyDialog*>(editor);
    if (keyDialog->result() == QDialog::Rejected)
        return;

    model->setData(index, keyDialog->getKeyName(), Qt::EditRole);
}
} // namespace gui
