#include "gui/labelEditor.h"
#include "ui_labelEditor.h"

#include <QCloseEvent>
#include <QMessageBox>
#include <set>

#include <iostream>

constexpr char emptyLabelName[] = "<label name>";

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

bool LabelEditor::isBlocked(int row) { return !mModel.item(row, 1)->isEditable(); }

void LabelEditor::on_buttonDelete_clicked()
{
    auto ids = mSelectionModel.selectedRows();
    if (ids.empty())
    {
        QMessageBox::information(this, "Info", "Please select a row to delete");
        return;
    }
    auto confirm
        = QMessageBox::question(this, "Delete?", "Are you sure you want to delete this label?");
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
    return {new QStandardItem(emptyLabelName), color, key};
}

LabelEditor::~LabelEditor()
{
    this->blockSignals(true);
    delete ui;
}

QList<QStringList> LabelEditor::getElements(int column) const
{
    QList<QStringList> lists;
    for (int r = 0; r < mModel.rowCount(); r++)
    {
        QStringList currList;
        auto index = mModel.index(r, 0);
        auto key = mModel.data(mModel.index(r, column)).toString();
        currList.push_back(key);

        if (mModel.hasChildren(index))
        {
            for (int c = 0; c < mModel.rowCount(index); c++)
            {
                key = mModel.data(mModel.index(c, column, index)).toString();
                currList.push_back(key);
            }
        }
        lists.push_back(currList);
    }
    return lists;
}

bool LabelEditor::checkDuplicate(int column, int level) const
{
    std::set<QString> set;
    auto insertOrExists = [&set](const QString& key) {
        if (key.size())
        {
            if (!set.count(key))
            {
                set.insert(key);
                return false;
            }
            else
                return true;
        }
        return false;
    };

    auto lists = getElements(column);
    for (const auto& list : lists)
    {
        for (int i = 0; i < list.size(); i++)
        {
            if (insertOrExists(list[i]))
            {
                return true;
            }

            // for level 0, only check first element
            if (level == 0)
                break;
        }

        // for level 1, reset set after every group
        if (level == 1)
            set.clear();
    }
    return false;
}

bool LabelEditor::checkEmpty(int column) const
{
    std::function<bool(const QString& key)> isEmpty;
    if (column == 0)
    {
        isEmpty = [](const QString& key) { return (key.size() == 0) || (key == emptyLabelName); };
    }
    else
        isEmpty = [](const QString& key) { return (key.size() == 0); };

    auto lists = getElements(column);
    for (const auto& list : lists)
        for (const auto& item : list)
            if (isEmpty(item))
                return true;

    return false;
}

bool LabelEditor::validate()
{
    // check duplicates in label column
    if (checkDuplicate(0, 0))
    {
        QMessageBox::information(this, "Info", "Label groups must be unique");
        return false;
    }

    if (checkDuplicate(0, 1))
    {
        QMessageBox::information(this, "Info", "Label groups must be unique");
        return false;
    }

    // check duplicates in hotkey column
    if (checkDuplicate(2))
    {
        QMessageBox::information(this, "Info", "Hotkeys must be unique");
        return false;
    }

    // check if labels are empty or not set
    if (checkEmpty(0))
    {
        QMessageBox::information(this, "Info", "Labels must be non-empty");
        return false;
    }

    return true;
}

void LabelEditor::on_accepted()
{
    if (!validate())
        return;

    auto configs = getLabelConfigs();
    emit accept();
}

int LabelEditor::keyStringToInt(const QString& key) const
{
    return QKeySequence::fromString(key)[0];
}

LabelGroupConfigs LabelEditor::getLabelConfigs() const
{
    LabelGroupConfigs configs;
    for (int r = 0; r < mModel.rowCount(); r++)
    {
        auto groupConfig = LabelGroupConfig();
        auto index = mModel.index(r, 0);
        if (mModel.hasChildren(index))
        {
            for (int c = 0; c < mModel.rowCount(index); c++)
            {
                auto name = mModel.data(mModel.index(c, 0, index)).toString().toStdString();
                auto color = mModel.data(mModel.index(c, 1, index)).toString().toStdString();
                auto key = mModel.data(mModel.index(c, 2, index)).toString().toStdString();
                /* groupConfig.insert({name, {color, key}}); */
            }
        }
        else
        {
            auto name = mModel.data(mModel.index(r, 0)).toString().toStdString();
            auto color = mModel.data(mModel.index(r, 1)).toString().toStdString();
            auto key = mModel.data(mModel.index(r, 2)).toString().toStdString();
            /* groupConfig.insert({name, {color, key}}); */
        }

        /* configs.push_back(std::move(groupConfig)); */
    }

    return configs;
}

} // namespace gui
