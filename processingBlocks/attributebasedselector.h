#ifndef ATTRIBUTEBASEDSELECTOR_H
#define ATTRIBUTEBASEDSELECTOR_H

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
