#include "habitrack/cache.h"

namespace ht
{
constexpr std::size_t ceil(std::size_t n, std::size_t k)
{
     return 1 + ((n - 1) / k);
}

Cache::Cache(std::size_t numElems, std::size_t maxChunkSize)
    : mNumElems(numElems)
    , mMaxChunkSize(maxChunkSize)
    , mNumChunks(ceil(mNumElems, mMaxChunkSize))
    , mRemainder(mNumElems % mMaxChunkSize)
{
}

std::size_t Cache::getNumElems() const { return mNumElems; }
std::size_t Cache::getMaxChunkSize() const { return mMaxChunkSize; }
std::size_t Cache::getNumChunks() const { return mNumChunks; }


std::size_t Cache::getChunkSize(std::size_t chunkIdx) const
{
    // definitely full chunks
    if (chunkIdx < getNumChunks() - 1)
        return mMaxChunkSize;
    
    // remainder for last chunk
    return mNumElems - (mNumChunks - 1) * mMaxChunkSize;
}

} // namespace ht
