#include "habitrack/idTranslator.h"

namespace ht::translate
{
std::vector<std::size_t> localToGlobal(
    const std::vector<std::vector<std::size_t>>& ids, const std::vector<std::size_t>& sizes)
{
    std::vector<std::size_t> globalIds;
    std::size_t cumSum = 0;
    for (std::size_t i = 0; i < ids.size(); i++)
    {
        for (auto id : ids[i])
            globalIds.push_back(id + cumSum);
        cumSum += sizes[i];
    }

    return globalIds;
}

/* std::vector<std::size_t> globalToLocal( */
/*     std::vector<std::size_t>& ids, const std::vector<std::size_t>& sizes) */
/* { */
/* } */

} // namespace ht::translate
