#ifndef ATTRIBUTEBASEDSELECTOR_H
#define ATTRIBUTEBASEDSELECTOR_H

#include <StereoVision/io/pointcloud_io.h>

#include "identityprocessor.h"


class AttributeBasedSelector : public IdentityProcessor
{
public:

    enum Comparator {
        Equal,
        Different,
        Greather,
        GreatherOrEqual,
        Smaller,
        SmallerOrEqual
    };

    /*!
     * \brief setupAttributeBasedSelector setup an attribute based selector
     * \param source the source point cloud (will be moved in case of success).
     * \param attributeName the name of the attribute
     * \param comparator if the attribute should be equal, different, greather or smaller than the value.
     * \param val the value the attribute is compared to.
     * \return a point cloud interface or nullptr in case of error.
     */
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupAttributeBasedSelector(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            std::string const& attributeName,
            Comparator comparator,
            StereoVision::IO::PointCloudGenericAttribute const& val);

    virtual bool gotoNext() override = 0;

protected:

    AttributeBasedSelector(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                           std::string const& attributeName,
                           StereoVision::IO::PointCloudGenericAttribute const& val);

    std::string _attributeName;
    StereoVision::IO::PointCloudGenericAttribute _comparisonVal;
};

#endif // ATTRIBUTEBASEDSELECTOR_H
