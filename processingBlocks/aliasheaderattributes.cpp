#include "aliasheaderattributes.h"

#include <set>

AliasHeaderAttributes::AliasHeaderAttributes(std::unique_ptr<StereoVision::IO::PointCloudHeaderInterface> source,
                                             AliasMap const& aliasMap)
{

    _initialAttributesSize = 0;
    _modifiedAttributeList = {};

    if (_src != nullptr) {
        std::vector<std::string> initialAttributes = _src->attributeList();
        _initialAttributesSize = initialAttributes.size();
        _modifiedAttributeList = initialAttributes;
    }

    std::set<std::string> attributes(_modifiedAttributeList.begin(), _modifiedAttributeList.end());

    for (const auto& [key, value] : _aliasMap) {

        if (attributes.count(key) > 0) {
            continue;
        }

        attributes.insert(key);
        _modifiedAttributeList.push_back(key);
    }

}

std::optional<StereoVision::IO::PointCloudGenericAttribute> AliasHeaderAttributes::getAttributeById(int id) const {

    if (id < 0 or id >= _modifiedAttributeList.size()) {
        return std::nullopt;
    }

    std::string attributeName = _modifiedAttributeList[id];

    if (_aliasMap.count(attributeName) > 0) {
        return _aliasMap.at(attributeName);
    }

    if (id < _initialAttributesSize) {
        return _src->getAttributeById(id);
    }

    return std::nullopt;

}

std::optional<StereoVision::IO::PointCloudGenericAttribute> AliasHeaderAttributes::getAttributeByName(const char* attributeName) const {

    if (_aliasMap.count(attributeName) > 0) {
        return _aliasMap.at(attributeName);
    }

    return _src->getAttributeByName(attributeName);

}

std::vector<std::string> AliasHeaderAttributes::attributeList() const {
    return _modifiedAttributeList;
}
