#ifndef GUI_LABELCONFIG_H
#define GUI_LABELCONFIG_H

#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace gui
{
struct LabelConfig
{
    std::string label;
    std::string color;
    int key;
};

// unique label name -> hotkey, color for one label group
// may contain at least two labels
using LabelGroupConfig = std::vector<LabelConfig>;

// unique group name -> group config
using LabelGroupConfigs = std::map<std::string, LabelGroupConfig>;

LabelGroupConfigs loadLabelGroupConfigs(const std::filesystem::path& configFile);
void saveLabelGroupConfigs(
    const std::filesystem::path& configFile, const LabelGroupConfigs& configs);

} // namespace gui

namespace cereal
{
template <class Archive>
void serialize(Archive& archive, gui::LabelConfig& config)
{
    archive(make_nvp("label", config.label), make_nvp("color", config.color),
        make_nvp("hotkey", config.key));
}
} // namespace cereal
#endif // GUI_LABELCONFIG_H
