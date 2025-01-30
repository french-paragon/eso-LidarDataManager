#include "pointsattributesfilters.h"


std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> PointsAttributesFilters::setupPointAttributeFiltering(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        bool removeColors,
        std::vector<std::string> const& excluded,
        bool removeAll) {

    if (source == nullptr) {
        return nullptr;
    }


    if (!removeColors and excluded.empty() and !removeAll) {
        return std::move(source);
    } else if (removeAll) {
        return std::unique_ptr<PointsAttributesFilters>(new PointsAttributesFilters(std::move(source), removeColors, {}, true));
    } else {
        return std::unique_ptr<PointsAttributesFilters>(new PointsAttributesFilters(std::move(source), removeColors, excluded, false));
    }

    return nullptr;

}

PointsAttributesFilters::PointsAttributesFilters(std::unique_ptr<PointCloudPointAccessInterface> &&source,
                                                 bool removeColors,
                                                 const std::vector<std::string> &excluded,
                                                 bool removeAll) :
    _src(std::move(source)),
    _filtered(excluded.begin(), excluded.end()),
    _filterAll(removeAll),
    _filterColor(removeColors)
{
    recomputeAttributes();
}

PointsAttributesFilters::~PointsAttributesFilters() {

}

StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> PointsAttributesFilters::getPointPosition() const {
    return _src->getPointPosition();
}

std::optional<StereoVision::IO::PtColor<StereoVision::IO::PointCloudGenericAttribute>> PointsAttributesFilters::getPointColor() const {

    if (_filterColor) {
        return std::nullopt;
    }

    return _src->getPointColor();
}

std::optional<StereoVision::IO::PointCloudGenericAttribute> PointsAttributesFilters::getAttributeById(int id) const {

    if (_filterAll) {
        return std::nullopt;
    }

    if (id >= 0 and id < _currentLineIdMatch.size()) {
        int subId = _currentLineIdMatch[id];
        return _src->getAttributeById(subId);
    }

    return std::nullopt;

}
std::optional<StereoVision::IO::PointCloudGenericAttribute> PointsAttributesFilters::getAttributeByName(const char* attributeName) const {

    if (_filterAll) {
        return std::nullopt;
    }

    if (_filtered.count(attributeName) > 0) {
        return std::nullopt;
    }

    return _src->getAttributeByName(attributeName);
}

std::vector<std::string> PointsAttributesFilters::attributeList() const {
    return _currentLineAttributes;
}

bool PointsAttributesFilters::gotoNext() {

    bool ok = _src->gotoNext();

    if (ok) {
        recomputeAttributes();
    }

    return ok;
}

void PointsAttributesFilters::recomputeAttributes() {

    _currentLineAttributes.clear();
    _currentLineIdMatch.clear();

    if (_filterAll) {
        return;
    }

    std::vector<std::string> srcAttributes = _src->attributeList();

    _currentLineAttributes.reserve(srcAttributes.size());
    _currentLineIdMatch.reserve(srcAttributes.size());

    for (int i = 0; i < srcAttributes.size(); i++) {

        if (_filtered.count(srcAttributes[i]) > 0) {
            continue;
        }

        _currentLineAttributes.push_back(srcAttributes[i]);
        _currentLineIdMatch.push_back(i);
    }
}
