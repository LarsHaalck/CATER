#include "habitrack/baseDecorator.h"

namespace ht
{
BaseDecorator::BaseDecorator(std::shared_ptr<ImageContainer> baseContainer)
    : ImageContainer(baseContainer->getData())
    , mBaseContainer(baseContainer)
{
}
BaseDecorator::~BaseDecorator()
{
}

cv::Mat BaseDecorator::at(std::size_t idx) const
{
    return mBaseContainer->at(idx);
}
} // namespace ht
