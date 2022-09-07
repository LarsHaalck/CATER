#ifndef CATER_BASE_CACHE_H
#define CATER_BASE_CACHE_H

#include <cstddef>
#include <tuple>
#include <vector>

namespace ct
{
class BaseCache
{
public:
    BaseCache(std::size_t numElems, std::size_t maxChunkSize, const std::vector<std::size_t>& ids);

    std::size_t getNumElems() const;
    std::size_t getMaxChunkSize() const;
    std::size_t getNumChunks() const;
    std::size_t getChunkSize(std::size_t chunkIdx) const;
    std::pair<std::size_t, std::size_t> getChunkBounds(std::size_t chunkIdx) const;

    virtual ~BaseCache() = default;

protected:
    std::size_t transformId(std::size_t idx) const;

private:
    std::size_t mNumElems;
    std::size_t mMaxChunkSize;
    std::size_t mNumChunks;
    std::size_t mRemainder;

    std::vector<std::size_t> mIds;
};
} // namespace ct

#endif // CATER_BASE_CACHE_H
