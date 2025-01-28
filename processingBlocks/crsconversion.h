#ifndef CRSCONVERSION_H
#define CRSCONVERSION_H

#include <memory>
#include <string>

#include <StereoVision/io/pointcloud_io.h>

struct pj_ctx;
struct PJconsts;

class CrsConversion : public StereoVision::IO::PointCloudPointAccessInterface
{
public:

    /*!
     * \brief setupCrsConversion try to setup a crs conversion
     * \param source a pointer to the source, will be moved to the output if return is not nullptr
     * \param inCrs the input crs
     * \param outCrs the output crs
     * \return a unique ptr to a PointCloudPointAccessInterface, or nullptr in case of error
     */
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupCrsConversion(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            std::string inCrs,
            std::string outCrs);

    ~CrsConversion();

    virtual StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> getPointPosition() const override;
    virtual std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> getPointColor() const override;

    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeById(int id) const override;
    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeByName(const char* attributeName) const override;

    virtual std::vector<std::string> attributeList() const override;

    virtual bool gotoNext() override;

protected:

    CrsConversion(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                  pj_ctx* projContext,
                  PJconsts* projTransform);

    void computeTransformedPoint();

    std::string _inCrs;
    std::string _outCrs;

    pj_ctx* _proj_ctx;
    PJconsts* _transform;

    std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> _src;

    StereoVision::IO::PtGeometry<double> _currentTransformedPosition;

};

#endif // CRSCONVERSION_H
