#ifndef HABITRACK_SPECIAL_GRAY_DECORATOR_H
#define HABITRACK_SPECIAL_GRAY_DECORATOR_H

#include "baseDecorator.h"

namespace ht
{
class SpecialGrayDecorator : public BaseDecorator
{
public:
    SpecialGrayDecorator(
        double red, double green, double blue, std::shared_ptr<ImageContainer> baseContainer);
    cv::Mat at(std::size_t idx) const override;

private:
    double mRed;
    double mGreen;
    double mBlue;
};
} // namespace ht
#endif // HABITRACK_SPECIAL_GRAY_DECORATOR_H
