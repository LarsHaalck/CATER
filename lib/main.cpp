#include <iostream>
#include <memory>

#include "habitrack/imageContainer.h"

int main()
{
    auto container = std::make_shared<ht::ImageContainer>(
        "/data/panorama/sevillaTest/vid1/imgs");
    auto cache = container->getCache(12);

    for (std::size_t i = 0; i < cache->getNumChunks(); i++)
    {
        auto currChunk = cache->getChunk(i);
        for (std::size_t k = 0; k < currChunk.size(); k++)
        {
            std::cout << currChunk[k].rows << ", " << currChunk[k].cols << std::endl;
        }
    }


    return 0;
}
