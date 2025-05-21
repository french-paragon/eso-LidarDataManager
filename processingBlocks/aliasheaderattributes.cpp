/*
 * This file is part of the LidarDataManager tool.
 * Copyright (c) 2025 Laurent Valentin Jospin <laurent.jospin@epfl.ch>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

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
