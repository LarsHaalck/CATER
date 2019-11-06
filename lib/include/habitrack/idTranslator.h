#ifndef HABITRACK_ID_TRANSLATOR_H
#define HABITRACK_ID_TRANSLATOR_H

#include <vector>

namespace ht::translate
{
// TODO: move this into Translator class
std::vector<std::size_t> localToGlobal(
    const std::vector<std::vector<std::size_t>>& ids, const std::vector<std::size_t>& sizes);
} // namespace ht

namespace ht
{
class Translator
{
public:
    Translator(const std::vector<std::size_t>& sizes);
    std::pair<std::size_t, std::size_t> globalToLocal(std::size_t idx);
    std::size_t localToGlobal(std::pair<std::size_t, std::size_t> vidFrameIdx);
private:
    std::vector<std::size_t> mCumSums;
};
} // namespace ht
#endif // HABITRACK_ID_TRANSLATOR_H
