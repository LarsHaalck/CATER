#ifndef GUI_LABELCONFIG_H
#define GUI_LABELCONFIG_H

#include <cereal/archives/json.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace gui
{
struct LabelConfig
{
    std::string color;
    int key;
    int id;
};

// unique label name -> hotkey, color for one label group
// may contain at least two labels
using LabelGroupConfig = std::unordered_map<std::string, LabelConfig>;

// unique group name -> group config
using LabelGroupConfigs = std::unordered_map<std::string, LabelGroupConfig>;


} // namespace gui

namespace cereal
{
template <class Archive>
void serialize(Archive& archive, gui::LabelConfig& config)
{
    archive(make_nvp("color", config.color), make_nvp("hotkey",config.key), make_nvp("id", config.id));
}
} // namespace cereal
#endif // GUI_LABELCONFIG_H
