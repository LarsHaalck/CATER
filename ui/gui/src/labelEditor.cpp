#include "gui/labelEditor.h"
#include "ui_labelEditor.h"

#include <set>
#include <QMessageBox>
#include <QCloseEvent>

#include <iostream>

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

    connect(ui->labelEditorButtonBox, SIGNAL(accepted()), this, SLOT(on_accepted())); 
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

bool LabelEditor::checkDuplicate() const
{
    std::set<QString> keySet;
    auto insertOrExists = [&](const QString& key)
    {
        if (key.size())
        {
            if (!keySet.count(key))
            {
                keySet.insert(key);
                return false;
            }
            else
                return true;
        }
        return false;
    };

    bool duplicate = false;
    for (int r = 0; r < mModel.rowCount(); r++)
    {
        auto index = mModel.index(r, 0);
        auto hotkey = mModel.data(mModel.index(r, 2)).toString();
        if (insertOrExists(hotkey))
        {
            duplicate = true;
            break;
        }
        if (mModel.hasChildren(index))
        {
            for (int c = 0; c < mModel.rowCount(index); c++)
            {
                hotkey = mModel.data(mModel.index(c, 2, index)).toString();
                if (insertOrExists(hotkey))
                {
                    duplicate = true;
                    break;
                }
            }
        }
    }
    return duplicate;
}

void LabelEditor::on_accepted()
{
    if (checkDuplicate())
    {
        QMessageBox::information(this, "Info", "Two or more hotkeys share a hotkey");
        return;
    }

    emit accept();
}


} // namespace gui
