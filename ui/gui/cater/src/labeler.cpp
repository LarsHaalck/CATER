#include "labeler.h"
#include "labelIO.h"

#include <fstream>
#include <spdlog/spdlog.h>
#include <QMessageBox>

namespace gui
{

enum class Mod
{
    CtrlAlt,
    Alt,
    Ctrl,
    None
};

void Labeler::init(std::size_t numImgs, const LabelGroupConfigs& config)
{
    mNumImgs = numImgs;
    mConfig = config;
    buildKeyMap();
}

void Labeler::buildKeyMap()
{
    for (const auto& group : mConfig)
    {
        auto groupName = group.first;
        for (std::size_t i = 0; i < group.second.size(); i++)
            mKeyMap.insert({group.second[i].key, {groupName, i}});
    }
}

void Labeler::initDefaultLabels()
{
    mLabels.resize(mNumImgs);
    Labels defaults;
    for (const auto& group : mConfig)
        defaults.insert({group.first, 0});

    // only init with default
    for (std::size_t i = 0; i < mNumImgs; i++)
    {
        auto& currLabels = mLabels[i];

        if (currLabels.empty())
        {
            currLabels = defaults;
            continue;
        }

        // iterate through all default labels
        for (const auto& def : defaults)
        {
            // check if the group of the default label is already set
            bool contains = false;
            for (const auto& label : currLabels)
            {
                if (def.first == label.first)
                {
                    contains = true;
                    break;
                }
            }

            // if not, then add it
            if (!contains)
                currLabels.insert(def);
        }
    }
}
std::pair<QString, QString> Labeler::getLabels(std::size_t frame) const
{
    if (mLabels.empty())
        return std::make_pair(QString(), QString());
    return getLabels(mLabels[frame]);
}

std::pair<QString, QString> Labeler::getStickyLabels() const { return getLabels(mStickyLabels); }

std::pair<QString, QString> Labeler::getLabels(const Labels& labels) const
{
    auto coloredCircle = [](const std::string& color) {
        return QString("<font color=\"") + QString::fromStdString(color) + QString("\">●</font>");
    };

    QString resShort, resLong;
    for (const auto& label : labels)
    {
        auto color = mConfig.at(label.first)[label.second].color;
        resShort += coloredCircle(color);
        resLong += labelIdToQString(label);
    }
    return std::make_pair(resShort, resLong);
}

QString Labeler::labelIdToQString(const LabelId& label) const
{
    auto name = QString::fromStdString(mConfig.at(label.first)[label.second].label);
    name = QString::fromStdString(label.first) + ": " + name + "\n";
    return name;
}

void Labeler::processKeyEvent(std::size_t frame, QKeyEvent* event)
{
    Mod mod = Mod::None;
    if (event->modifiers().testFlag(Qt::ControlModifier)
        && event->modifiers().testFlag(Qt::AltModifier))
        mod = Mod::CtrlAlt;
    else if (event->modifiers().testFlag(Qt::ControlModifier))
        mod = Mod::Ctrl;
    else if (event->modifiers().testFlag(Qt::AltModifier))
        mod = Mod::Alt;

    auto key = event->key();
    if (key == Qt::Key_Backspace)
        clearSticky();
    else if (mKeyMap.count(key))
    {
        auto groupName = mKeyMap[key].first;
        auto labelId = mKeyMap[key].second;
        auto label = std::make_pair(groupName, labelId);

        // on continous label, do nothing else
        if (mod == Mod::Ctrl)
        {
            labelContinous(frame, label);
            return;
        }

        insertLabel(label, mLabels[frame]);
        if (mod == Mod::Alt)
            makeSticky(label);
    }
}

bool Labeler::isSticky(const LabelId& label) { return mStickyLabels.count(label); }

// check not only if group exists but also if the label is equal
bool Labeler::isStickyEqual(const LabelId& label)
{
    return (isSticky(label) && ((*mStickyLabels.find(label)).second == label.second));
}

void Labeler::processSticky(std::size_t frame)
{
    for (const auto& sticky : mStickyLabels)
        insertLabel(sticky, mLabels[frame]);
}

void Labeler::makeSticky(const LabelId& label)
{
    if (isStickyEqual(label))
        mStickyLabels.erase(label);
    else
        insertLabel(label, mStickyLabels);
}

void Labeler::labelContinous(std::size_t frame, const LabelId& label)
{
    // find the last occurence of this label instantion before the curent frame
    int idx = -1;
    for (int i = static_cast<int>(frame) - 1; i >= 0; i--)
    {
        if (auto it = mLabels[i].find(label);
            it != std::end(mLabels[i]) && (*it).second == label.second)
        {
            idx = i;
            break;
        }
    }

    // not found, nothing to to do
    if (idx < 0)
        return;

    QString msg
        = QString("Do you want to label all frames between %1 and %2 with label: %3")
              .arg(QString::number(idx + 1), QString::number(frame + 1), labelIdToQString(label));

    QMessageBox msgBox;
    msgBox.setText("Label all frames");
    msgBox.setInformativeText(msg);
    msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
    msgBox.setDefaultButton(QMessageBox::No);
    int ret = msgBox.exec();

    if (ret != QMessageBox::Yes)
        return;

    // label all frames from this last instantion exlusive left (because it was already set) and 
    // inclusive right (because frame label was not set in processKeyEvent)
    for (int i = idx + 1; i <= static_cast<int>(frame); i++)
        insertLabel(label, mLabels[i]);
}

void Labeler::insertLabel(const LabelId& label, Labels& labels)
{
    // erase if already existing
    if (auto it = labels.find(label); it != std::end(labels))
        labels.erase(it);
    labels.insert(label);
}

void processSticky(std::size_t frame);

void Labeler::save(const std::filesystem::path& labelFile)
{
    std::ofstream stream(labelFile.string(), std::ofstream::out);
    ct::io::checkStream(stream, labelFile);
    {
        cereal::JSONOutputArchive archive(stream);
        archive(mLabels);
    }
}

void Labeler::load(const std::filesystem::path& labelFile)
{
    std::ifstream stream(labelFile.string(), std::ofstream::in);
    ct::io::checkStream(stream, labelFile);
    {
        cereal::JSONInputArchive archive(stream);
        archive(mLabels);
    }
}

} // namespace gui
