#include "gui/colorDelegate.h"

#include <QColorDialog>
#include <QPainter>

#include <iostream>

#include <QEvent>

namespace gui
{
ColorDelegate::ColorDelegate(QObject* parent)
    : QItemDelegate(parent)
{
}

QWidget* ColorDelegate::createEditor(
    QWidget* parent, const QStyleOptionViewItem&, const QModelIndex&) const
{
    auto editor = new QColorDialog(parent);
    editor->setModal(true);
    return editor;
}

void ColorDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    auto variant = index.model()->data(index, Qt::EditRole);
    auto color = variant.value<QColor>();
    auto colorDialog = static_cast<QColorDialog*>(editor);

    if (variant.toString().size())
        colorDialog->setCurrentColor(color);
}

void ColorDelegate::setModelData(
    QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    auto colorDialog = static_cast<QColorDialog*>(editor);
    if (colorDialog->result() == QDialog::Rejected)
        return;

    auto color = colorDialog->currentColor();
    model->setData(index, color.name(), Qt::EditRole);
}

void ColorDelegate::paint(
    QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    if (option.state & QStyle::State_Selected)
        painter->fillRect(option.rect, option.palette.highlight());

    auto variant = index.model()->data(index, Qt::DisplayRole);

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);

    auto center = option.rect.center();
    auto height = option.rect.height() / 2;
    if (!variant.toString().size())
    {
        painter->setBrush(Qt::Dense6Pattern);
        painter->drawEllipse(center, height, height);
    }
    else
    {
        auto color = variant.value<QColor>();
        painter->setBrush(color);
        painter->drawEllipse(center, height, height);
    }

    painter->restore();

}
} // namespace gui
