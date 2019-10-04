#include <iostream>
#include <memory>

#include "habitrack/imageContainer.h"
#include "habitrack/resizeDecorator.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

int main()
{
    auto container = std::make_shared<ht::ImageContainer>(
        "/data/panorama/sevillaTest/vid1/imgs");
    /* auto cache = container->getCache(12); */
    /* for (std::size_t i = 0; i < cache->getNumChunks(); i++) */
    /* { */
    /*     auto currChunk = cache->getChunk(i); */
    /*     for (std::size_t k = 0; k < currChunk.size(); k++) */
    /*     { */
    /*         std::cout << currChunk[k].rows << ", " << currChunk[k].cols << std::endl; */
    /*     } */
    /* } */

    {
        auto resizeContainer = container->resize(0.5)->gray()->resize(0.5);
        for (int i = 0; i < 10; i++)
        {
            cv::imshow("test", resizeContainer->at(i));
            cv::waitKey(0);
        }
    }


    return 0;
}
