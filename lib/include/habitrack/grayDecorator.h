#ifndef HABITRACK_GRAY_DECORATOR_H
#define HABITRACK_GRAY_DECORATOR_H

#include "habitrack/baseDecorator.h"

namespace ht
{
class GrayDecorator : public BaseDecorator
{
public:
    GrayDecorator(std::shared_ptr<ImageContainer> baseContainer);
    cv::Mat at(std::size_t idx, ImageType imageType) const override;
};
} // namespace ht
#endif // HABITRACK_GRAY_DECORATOR_H

