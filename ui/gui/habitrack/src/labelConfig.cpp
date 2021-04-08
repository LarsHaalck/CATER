#include "labelConfig.h"

#include <cereal/archives/json.hpp>
#include <filesystem>
#include <fstream>

namespace gui
{
LabelGroupConfigs loadLabelGroupConfigs(const std::filesystem::path& configFile)
{
    LabelGroupConfigs configs;
    std::ifstream stream(configFile.string(), std::ifstream::in);
    {
        cereal::JSONInputArchive archive(stream);
        archive(configs);
    }
    return configs;
}

void saveLabelGroupConfigs(
    const std::filesystem::path& configFile, const LabelGroupConfigs& configs)
{
    std::ofstream stream(configFile.string(), std::ofstream::out);
    {
        cereal::JSONOutputArchive archive(stream);
        archive(configs);
    }
}
} // namespace gui
