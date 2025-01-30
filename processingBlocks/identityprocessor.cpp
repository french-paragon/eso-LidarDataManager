#include "identityprocessor.h"

IdentityProcessor::IdentityProcessor(std::unique_ptr<PointCloudPointAccessInterface> &&source) :
    _src(std::move(source))
{

}

StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> IdentityProcessor::getPointPosition() const {
    return _src->getPointPosition();
}
std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> IdentityProcessor::getPointColor() const {
    return _src->getPointColor();
}

std::optional<StereoVision::IO::PointCloudGenericAttribute> IdentityProcessor::getAttributeById(int id) const {
    return _src->getAttributeById(id);
}
std::optional<StereoVision::IO::PointCloudGenericAttribute> IdentityProcessor::getAttributeByName(const char* attributeName) const {
    return _src->getAttributeByName(attributeName);
}

std::vector<std::string> IdentityProcessor::attributeList() const {
    return _src->attributeList();
}

bool IdentityProcessor::gotoNext() {
    return _src->gotoNext();
}
