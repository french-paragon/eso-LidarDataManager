#ifndef INDENTITYPROCESSOR_H
#define INDENTITYPROCESSOR_H

#include <StereoVision/io/pointcloud_io.h>

/*!
 * \brief The IdentityProcessor class represent a processor block which return the point cloud as is.
 *
 * Many processor will impact only the geometry, or only the color, or only the attributes, or only skip points.
 * To avoid having to rewrite a bunch of identity functions everytime, they can inherit this class.
 *
 * Note that the functions assume that the source is a valid interface, and not nullptr.
 *
 * To deal with potential nullptr source, it is recommanded to use a factory pattern for the child classes,
 * and not return a processor in case the source is not valid.
 */
class IdentityProcessor : public StereoVision::IO::PointCloudPointAccessInterface
{
public:
    IdentityProcessor(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source);

    virtual StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> getPointPosition() const override;
    virtual std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> getPointColor() const override;

    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeById(int id) const override;
    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeByName(const char* attributeName) const override;

    virtual std::vector<std::string> attributeList() const override;

    virtual bool gotoNext() override;

protected:
    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> _src;
};

#endif // INDENTITYPROCESSOR_H
