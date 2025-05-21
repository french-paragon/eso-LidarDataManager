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

bool IdentityProcessor::hasData() const {
    return _src->hasData();
}
