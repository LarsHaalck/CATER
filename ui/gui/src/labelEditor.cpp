#include "gui/labelEditor.h"
#include "ui_labelEditor.h"

#include <iostream>
#include <QMessageBox>

namespace gui
{
LabelEditor::LabelEditor(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::LabelEditor)
    , mModel(0, 0, this)
    , mSelectionModel(&mModel)
{
    ui->setupUi(this);
    ui->treeView->setModel(&mModel);
    ui->treeView->setSelectionModel(&mSelectionModel);
    ui->treeView->setHeaderHidden(false);

    ui->treeView->setItemDelegateForColumn(1, &mColorDelegate);
    ui->treeView->setItemDelegateForColumn(2, &mKeyDelegate);

    mModel.setHorizontalHeaderLabels({"Label Name", "Color", "Hot-Key"});
}

void LabelEditor::on_buttonNewGroup_clicked()
{
    QStandardItem* parent = mModel.invisibleRootItem();
    parent->appendRow(getDefaultItems());
}

void LabelEditor::on_buttonNewLabel_clicked()
{
    auto ids = mSelectionModel.selectedRows();
    if (ids.empty())
    {
        QMessageBox::information(this, "Info", "Please select one row or create a group first");
        return;
    }

    if (ids.size() > 1)
    {
        QMessageBox::information(this, "Info", "Please select only one row");
        return;
    }

    auto id = ids[0];

    // check if id has parent (i.e. a label) and select parent (i.e. label group then)
    if (id.parent().isValid())
        id = id.parent();

    auto row = id.row();
    if (!isBlocked(row))
    {
        auto confirm = QMessageBox::question(this, "Override Parent?",
            "Adding sub-labels will reset color and hotkey for parent. Continue?");
        if (confirm == QMessageBox::No)
            return;
        blockResetItem(row);
    }

    auto parent = mModel.item(row);
    parent->appendRow(getDefaultItems());
}

void LabelEditor::blockResetItem(int row)
{
    // reset color
    mModel.item(row, 1)->setEditable(false);
    mModel.item(row, 1)->setText("");

    // reset key
    mModel.item(row, 2)->setEditable(false);
    mModel.item(row, 2)->setText("");
}

void LabelEditor::unblockItem(int row)
{
    mModel.item(row, 1)->setEditable(true);
    mModel.item(row, 2)->setEditable(true);
}

bool LabelEditor::isBlocked(int row)
{
    return !mModel.item(row, 1)->isEditable();
}

void LabelEditor::on_buttonDelete_clicked()
{
    auto ids = mSelectionModel.selectedRows();
    if (ids.empty())
    {
        QMessageBox::information(this, "Info", "Please select a row to delete");
        return;
    }
    auto confirm = QMessageBox::question(this, "Delete?", "Are you sure you want to delete this label?");
    if (confirm == QMessageBox::No)
        return;


    for (auto item : ids)
    {
        mModel.removeRow(item.row(), item.parent());

        if (item.parent().isValid())
        {
            auto row = item.parent().row();
            unblockItem(row);
        }
    }
}

QList<QStandardItem*> LabelEditor::getDefaultItems() const
{
    auto color = new QStandardItem(QString(""));
    auto key = new QStandardItem(QString(""));
    QString name = "<label name>";

    return {new QStandardItem(name), color, key};
}

LabelEditor::~LabelEditor()
{
    this->blockSignals(true);
    delete ui;
}


} // namespace gui
