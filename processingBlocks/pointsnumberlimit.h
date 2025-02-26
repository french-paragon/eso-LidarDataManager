#ifndef POINTSNUMBERLIMIT_H
#define POINTSNUMBERLIMIT_H

#include <StereoVision/io/pointcloud_io.h>

#include "./identityprocessor.h"

/*!
 * \brief The PointsNumberLimit class subsample a point cloud.
 *
 * This class return a set maximum number of points, and can skip points in the original reader.
 */
class PointsNumberLimit : public IdentityProcessor
{
public:
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupPointNumberLimit(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            size_t limit,
            size_t step = 1);

    PointsNumberLimit(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                      size_t limit,
                      size_t step);
    ~PointsNumberLimit();

    virtual bool gotoNext() override;

protected:

    int _count;
    int _limit;
    int _step;

};

#endif // POINTSNUMBERLIMIT_H
