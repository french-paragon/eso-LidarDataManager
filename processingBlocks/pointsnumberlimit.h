#ifndef POINTSNUMBERLIMIT_H
#define POINTSNUMBERLIMIT_H

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

#include "./identityprocessor.h"

/*!
 * \brief The PointsNumberLimit class subsample a point cloud.
 *
 * This class return a set maximum number of points, and can skip points in the original reader.
 */
class PointsNumberLimit : public IdentityProcessor
{
public:
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupPointNumberLimit(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            size_t limit,
            size_t step = 1);

    PointsNumberLimit(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                      size_t limit,
                      size_t step);
    ~PointsNumberLimit();

    virtual bool gotoNext() override;

protected:

    int _count;
    int _limit;
    int _step;

};

#endif // POINTSNUMBERLIMIT_H
