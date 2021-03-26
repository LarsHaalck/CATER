#ifndef GUI_LABELER_H
#define GUI_LABELER_H

#include <filesystem>
#include <unordered_map>
#include <unordered_set>

#include <QKeyEvent>

#include "gui/labelConfig.h"
#include "util/pairHash.h"

namespace gui
{
using LabelId = std::pair<std::string, int>;
using Labels = std::unordered_set<LabelId, ht::hash>;

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
    bool isSticky(const LabelId& label);
    void insertLabel(const LabelId& label, Labels& labels);
    std::pair<QString, QString> getLabels(const Labels& labels) const;

private:
    std::size_t mNumImgs;
    LabelGroupConfigs mConfig;

    std::unordered_map<int, LabelId> mKeyMap;
    Labels mStickyLabels;

    std::vector<Labels> mLabels;
};
} // namespace gui

namespace cereal
{
//! Serializing for std::pair
template <class Archive>
void serialize(Archive& archive, std::pair<std::string, int>& labelId)
{
    archive(make_nvp("label", labelId.first));
    archive(make_nvp("value:", labelId.second));
}

template <class Archive, class T>
void serialize(Archive& archive, std::pair<std::size_t, T>& test)
{
    archive(make_nvp("frame", test.first));
    archive(make_nvp("labels", test.second));
}

template <class Archive, class T>
void save(Archive& archive, const std::vector<T>& vec)
{
    archive(make_size_tag(static_cast<size_type>(vec.size())));
    for (std::size_t i = 0; i < vec.size(); i++)
        archive(std::make_pair(i, vec[i]));
}

template <class Archive, class T>
void load(Archive& archive, std::vector<T>& vec)
{
    size_type size;
    archive(make_size_tag(size));
    vec.resize(static_cast<std::size_t>(size));

    for(auto&& v : vec)
    {
        auto pair = std::pair<std::size_t, T>();
        archive(pair);
        v = pair.second;
    }
}

} // namespace cereal
#endif // GUI_LABELER_H
