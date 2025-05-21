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

#include "crsconversion.h"

#include <proj.h>

#include <cmath>



std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> CrsConversion::setupCrsConversion(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        std::string inCrs,
        std::string outCrs) {

    if (source == nullptr) {
        return nullptr;
    }

    if (inCrs == outCrs) {
        return std::move(source);
    }

    PJ_CONTEXT* proj_ctx;
    PJ* transform;

    proj_ctx = proj_context_create();

    if (proj_ctx == 0) {
        return nullptr;
    }

    transform = proj_create_crs_to_crs(proj_ctx, inCrs.c_str(), outCrs.c_str(), nullptr);

    if (transform == 0) {
        return nullptr;
    }

    return std::unique_ptr<CrsConversion>(new CrsConversion(std::move(source),proj_ctx,transform));

}

CrsConversion::CrsConversion(std::unique_ptr<PointCloudPointAccessInterface> && source,
                             pj_ctx* projContext,
                             PJconsts* projTransform) :
    IdentityProcessor(std::move(source)),
    _proj_ctx(projContext),
    _transform(projTransform)
{
    computeTransformedPoint();
}
CrsConversion::~CrsConversion() {
    proj_destroy(_transform);
    proj_context_destroy(_proj_ctx);
}

StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> CrsConversion::getPointPosition() const {

    StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> ret;

    ret.x = _currentTransformedPosition.x;
    ret.y = _currentTransformedPosition.y;
    ret.z = _currentTransformedPosition.z;

    return ret;
}

bool CrsConversion::gotoNext() {

    bool ok = _src->gotoNext();

    if (ok) {
        computeTransformedPoint();
    }

    return ok;
}

void CrsConversion::computeTransformedPoint() {

    _currentTransformedPosition = _src->castedPointGeometry<double>();

    constexpr int n = 1;
    constexpr int delta_x = 1;
    constexpr int delta_y = 1;
    constexpr int delta_z = 1;

    proj_trans_generic(_transform, PJ_FWD,
                       &_currentTransformedPosition.x, delta_x, n,
                       &_currentTransformedPosition.y, delta_y, n,
                       &_currentTransformedPosition.z, delta_z, n,
                       nullptr, 0, 0);

}
