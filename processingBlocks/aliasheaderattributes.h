#ifndef ALIASHEADERATTRIBUTES_H
#define ALIASHEADERATTRIBUTES_H

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
