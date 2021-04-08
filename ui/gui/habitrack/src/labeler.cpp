#include "labeler.h"

#include <fstream>
#include <spdlog/spdlog.h>

#include "labelIO.h"

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
        auto name = QString::fromStdString(mConfig.at(label.first)[label.second].label);
        resShort += coloredCircle(color);
        resLong += QString::fromStdString(label.first) + ": " + name + "\n";
    }
    return std::make_pair(resShort, resLong);
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
    ht::io::checkStream(stream, labelFile);
    {
        cereal::JSONOutputArchive archive(stream);
        archive(mLabels);
    }
}

void Labeler::load(const std::filesystem::path& labelFile)
{
    std::ifstream stream(labelFile.string(), std::ofstream::in);
    ht::io::checkStream(stream, labelFile);
    {
        cereal::JSONInputArchive archive(stream);
        archive(mLabels);
    }
}

} // namespace gui
