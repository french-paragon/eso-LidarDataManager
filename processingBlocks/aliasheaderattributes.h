#ifndef ALIASHEADERATTRIBUTES_H
#define ALIASHEADERATTRIBUTES_H

#include <map>
#include <string>

#include <StereoVision/io/pointcloud_io.h>

class AliasHeaderAttributes : public StereoVision::IO::PointCloudHeaderInterface
{
public:
    using AliasMap = std::map<std::string, StereoVision::IO::PointCloudGenericAttribute>;

    AliasHeaderAttributes(std::unique_ptr<StereoVision::IO::PointCloudHeaderInterface> source,
                          AliasMap const& aliasMap);

    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeById(int id) const override;
    virtual std::optional<StereoVision::IO::PointCloudGenericAttribute> getAttributeByName(const char* attributeName) const override;

    virtual std::vector<std::string> attributeList() const override;

protected:

    int _initialAttributesSize;
    std::vector<std::string> _modifiedAttributeList;

    std::unique_ptr<StereoVision::IO::PointCloudHeaderInterface> _src;
    AliasMap _aliasMap;
};

#endif // ALIASHEADERATTRIBUTES_H
