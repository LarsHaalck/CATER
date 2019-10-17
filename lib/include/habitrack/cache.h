#ifndef HABITRACK_CACHE_H
#define HABITRACK_CACHE_H

#include <cstddef>
#include <tuple>

namespace ht
{
class Cache
{
public:
    Cache(std::size_t numElems, std::size_t maxChunkSize, bool keyFramesOnly = false);

    std::size_t getNumElems() const;
    std::size_t getMaxChunkSize() const;
    std::size_t getNumChunks() const;
    std::size_t getChunkSize(std::size_t chunkIdx) const;
    std::pair<std::size_t, std::size_t> getChunkBounds(std::size_t chunkIdx) const;

    virtual ~Cache() = default;
private:
    std::size_t mNumElems;
    std::size_t mMaxChunkSize;
    std::size_t mNumChunks;
    std::size_t mRemainder;
protected:
    bool mKeyFramesOnly;
};
} // namespace ht

#endif // HABITRACK_CACHE_H

