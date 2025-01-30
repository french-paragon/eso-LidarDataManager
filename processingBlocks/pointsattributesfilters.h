#ifndef POINTSATTRIBUTESFILTERS_H
#define POINTSATTRIBUTESFILTERS_H

#include <string>
#include <set>
#include <vector>

#include <StereoVision/io/pointcloud_io.h>

#include "./identityprocessor.h"

class PointsAttributesFilters : public IdentityProcessor
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
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupPointAttributeFiltering(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            bool removeColors,
            std::vector<std::string> const& excluded,
            bool removeAll);

    ~PointsAttributesFilters();

    virtual std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> getPointColor() const override;

    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeById(int id) const override;
    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeByName(const char* attributeName) const override;

    virtual std::vector<std::string> attributeList() const override;

    virtual bool gotoNext() override;

protected:

    PointsAttributesFilters(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                            bool removeColors,
                            std::vector<std::string> const& excluded,
                            bool removeAll);

    void recomputeAttributes();

    std::set<std::string> _filtered;
    const bool _filterColor;
    const bool _filterAll;

    std::vector<std::string> _currentLineAttributes;
    std::vector<int> _currentLineIdMatch;

};

#endif // POINTSATTRIBUTESFILTERS_H
