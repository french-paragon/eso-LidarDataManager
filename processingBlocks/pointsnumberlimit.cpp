#include "pointsnumberlimit.h"


std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> PointsNumberLimit::setupPointNumberLimit(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        size_t limit,
        size_t step) {

    if (source == nullptr) {
        return nullptr;
    }

    return std::unique_ptr<PointsNumberLimit>(new PointsNumberLimit(std::move(source), limit, step));

}
PointsNumberLimit::PointsNumberLimit(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                                     size_t limit,
                                     size_t step) :
    IdentityProcessor(std::move(source)),
    _count(0),
    _limit(limit),
    _step(step)
{

}

PointsNumberLimit::~PointsNumberLimit() {

}

bool PointsNumberLimit::gotoNext() {
    if (_count >= _limit) {
        return false;
    }

    bool ok = true;

    for (int i = 0; i < _step; i++) {
        ok = IdentityProcessor::gotoNext();
        if (!ok) {
            return false;
        }
    }
    _count++;
    return true;
}
