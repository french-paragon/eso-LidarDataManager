#ifndef REGIONOFINTERESTSELECTOR_H
#define REGIONOFINTERESTSELECTOR_H

#include <StereoVision/geometry/rotations.h>
#include <StereoVision/io/pointcloud_io.h>

#include "./identityprocessor.h"

class RegionOfInterestSelector : public IdentityProcessor
{
public:

    /*!
     * \brief setupCrsConversion try to setup a crs conversion
     * \param source a pointer to the source, will be moved to the output if return is not nullptr
     * \param removeColors remove the colors of the points
     * \param excluded the excluded attributes
     * \param removeAll remove all attributes
     * \return a unique ptr to a PointCloudPointAccessInterface, or nullptr in case of error
     */
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupRoiSelection(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            std::string const& definition);

    ~RegionOfInterestSelector();

    virtual bool gotoNext() override;

protected:

    RegionOfInterestSelector(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                             StereoVision::Geometry::AffineTransform<double> const& transform,
                             std::array<double, 3> const& extents);

    StereoVision::Geometry::AffineTransform<double> _transform;
    std::array<double, 3> _extends;
};

#endif // REGIONOFINTERESTSELECTOR_H
