#ifndef HABITRACK_ID_TRANSLATOR_H
#define HABITRACK_ID_TRANSLATOR_H

#include <vector>

namespace ht::translate
{
// TODO:
// maybe use class with sizes as constructur argument and precalc exclusive_scan
std::vector<std::size_t> localToGlobal(
    const std::vector<std::vector<std::size_t>>& ids, const std::vector<std::size_t>& sizes);

/* std::vector<std::size_t> globalToLocal( */
/*     std::vector<std::size_t>& ids, const std::vector<std::size_t>& sizes); */

} // namespace ht
#endif // HABITRACK_ID_TRANSLATOR_H
