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

#include "pointsnumberlimit.h"


std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> PointsNumberLimit::setupPointNumberLimit(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        size_t limit,
        size_t step) {

    if (source == nullptr) {
        return nullptr;
    }

    return std::unique_ptr<PointsNumberLimit>(new PointsNumberLimit(std::move(source), limit, step));

}
PointsNumberLimit::PointsNumberLimit(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                                     size_t limit,
                                     size_t step) :
    IdentityProcessor(std::move(source)),
    _count(0),
    _limit(limit),
    _step(step)
{

}

PointsNumberLimit::~PointsNumberLimit() {

}

bool PointsNumberLimit::gotoNext() {
    if (_count >= _limit) {
        return false;
    }

    bool ok = true;

    for (int i = 0; i < _step; i++) {
        ok = IdentityProcessor::gotoNext();
        if (!ok) {
            return false;
        }
    }
    _count++;
    return true;
}
