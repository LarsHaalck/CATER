#include "habitrack/baseCache.h"

namespace ht
{
constexpr std::size_t ceil(std::size_t n, std::size_t k)
{
    if (n > 0)
        return 1 + ((n - 1) / k);
    return 0;
}

BaseCache::BaseCache(
    std::size_t numElems, std::size_t maxChunkSize, const std::vector<std::size_t>& ids)
    : mNumElems(ids.empty() ? numElems : ids.size())
    , mMaxChunkSize(maxChunkSize ? maxChunkSize : mNumElems)
    , mNumChunks(ceil(mNumElems, mMaxChunkSize))
    , mRemainder(mNumElems % mMaxChunkSize)
    , mIds(ids)
{
}

std::size_t BaseCache::getNumElems() const { return mNumElems; }
std::size_t BaseCache::getMaxChunkSize() const { return mMaxChunkSize; }
std::size_t BaseCache::getNumChunks() const { return mNumChunks; }

std::size_t BaseCache::transformId(std::size_t idx) const
{
    if (mIds.empty())
        return idx;
    return mIds[idx];
}
std::size_t BaseCache::getChunkSize(std::size_t chunkIdx) const
{
    // definitely full chunks
    if (chunkIdx < getNumChunks() - 1)
        return mMaxChunkSize;

    // remainder for last chunk
    return mNumElems - (mNumChunks - 1) * mMaxChunkSize;
}

std::pair<std::size_t, std::size_t> BaseCache::getChunkBounds(std::size_t chunkIdx) const
{
    auto start = chunkIdx * getChunkSize(chunkIdx - 1);
    return {start, start + getChunkSize(chunkIdx)};
}
} // namespace ht
