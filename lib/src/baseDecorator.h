#ifndef HABITRACK_BASE_DECORATOR_H
#define HABITRACK_BASE_DECORATOR_H

#include "habitrack/imageContainer.h"

namespace ht
{
class BaseDecorator : public ImageContainer
{
public:
    BaseDecorator(std::shared_ptr<ImageContainer> baseContainer);
    virtual ~BaseDecorator();

    virtual cv::Size getImgSize() const override;
    virtual cv::Mat at(std::size_t idx) const override;

private:
    std::shared_ptr<ImageContainer> mBaseContainer;
};
} // namespace ht

#endif // HABITRACK_BASE_DECORATOR_H
