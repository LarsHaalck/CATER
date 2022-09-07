#ifndef GUI_LABELER_H
#define GUI_LABELER_H

#include <filesystem>
#include <set>
#include <unordered_map>

#include <QKeyEvent>

#include "labelConfig.h"
#include <cater/util/pairHash.h>

namespace gui
{
using LabelId = std::pair<std::string, int>;

struct compare_struct
{
    bool operator()(
        const std::pair<std::string, int>& lhs, const std::pair<std::string, int>& rhs) const
    {
        return (lhs.first < rhs.first);
    }
};

using Labels = std::set<LabelId, compare_struct>;

class Labeler
{
public:
    Labeler() = default;
    void init(std::size_t numImgs, const LabelGroupConfigs& config);
    void initDefaultLabels();

    void processKeyEvent(std::size_t frame, QKeyEvent* event);
    void processSticky(std::size_t frame);
    void clearSticky() { mStickyLabels.clear(); }

    std::pair<QString, QString> getLabels(std::size_t frame) const;
    std::pair<QString, QString> getStickyLabels() const;

    void save(const std::filesystem::path& labelFile);
    void load(const std::filesystem::path& labelFile);

private:
    void buildKeyMap();
    void makeSticky(const LabelId& label);
    void labelContinous(std::size_t frame, const LabelId& label);
    bool isSticky(const LabelId& label);
    bool isStickyEqual(const LabelId& label);
    void insertLabel(const LabelId& label, Labels& labels);
    std::pair<QString, QString> getLabels(const Labels& labels) const;
    QString labelIdToQString(const LabelId& label) const;

private:
    std::size_t mNumImgs;
    LabelGroupConfigs mConfig;

    std::unordered_map<int, LabelId> mKeyMap;
    Labels mStickyLabels;

    std::vector<Labels> mLabels;
};
} // namespace gui
#endif // GUI_LABELER_H
