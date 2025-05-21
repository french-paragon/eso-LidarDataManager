#ifndef CRSCONVERSION_H
#define CRSCONVERSION_H

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

#include <memory>
#include <string>

#include <StereoVision/io/pointcloud_io.h>

#include "identityprocessor.h"

struct pj_ctx;
struct PJconsts;

class CrsConversion : public IdentityProcessor
{
public:

    /*!
     * \brief setupCrsConversion try to setup a crs conversion
     * \param source a pointer to the source, will be moved to the output if return is not nullptr
     * \param inCrs the input crs
     * \param outCrs the output crs
     * \return a unique ptr to a PointCloudPointAccessInterface, or nullptr in case of error
     */
    static std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> setupCrsConversion(
            std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
            std::string inCrs,
            std::string outCrs);

    ~CrsConversion();

    virtual StereoVision::IO::PtGeometry<StereoVision::IO::PointCloudGenericAttribute> getPointPosition() const override;

    virtual bool gotoNext() override;

protected:

    CrsConversion(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> && source,
                  pj_ctx* projContext,
                  PJconsts* projTransform);

    void computeTransformedPoint();

    std::string _inCrs;
    std::string _outCrs;

    pj_ctx* _proj_ctx;
    PJconsts* _transform;

    StereoVision::IO::PtGeometry<double> _currentTransformedPosition;

};

#endif // CRSCONVERSION_H
