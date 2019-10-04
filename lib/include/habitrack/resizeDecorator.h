#ifndef HABITRACK_RESIZE_DECORATOR_H
#define HABITRACK_RESIZE_DECORATOR_H

#include "habitrack/baseDecorator.h"

namespace ht
{
class ResizeDecorator : public BaseDecorator
{
public:
    ResizeDecorator(double scaleX, double scaleY,
        std::shared_ptr<ImageContainer> baseContainer);

    cv::Mat at(std::size_t idx) const override;
private:
    double mScaleX;
    double mScaleY;
};
} // namespace ht
#endif // HABITRACK_RESIZE_DECORATOR_H

