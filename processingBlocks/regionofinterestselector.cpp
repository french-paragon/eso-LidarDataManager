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

#include "regionofinterestselector.h"
#include <sstream>

std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> RegionOfInterestSelector::setupRoiSelection(
        std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> & source,
        std::string const& definition) {

    if (source == nullptr) {
        return nullptr;
    }

    std::istringstream reader(definition);
    std::string s;
    bool ok = true;

    double x;
    double y;
    double z;

    double dx;
    double dy;
    double dz;

    double rx;
    double ry;
    double rz;

    try {

        std::getline(reader, s, ',');
        x = stod(s);
        std::getline(reader, s, ',');
        y = stod(s);
        std::getline(reader, s, ',');
        z = stod(s);

        std::getline(reader, s, ',');
        dx = stod(s);
        std::getline(reader, s, ',');
        dy = stod(s);
        std::getline(reader, s, ',');
        dz = stod(s);

        std::getline(reader, s, ',');
        rx = stod(s);
        std::getline(reader, s, ',');
        ry = stod(s);
        std::getline(reader, s, ',');
        rz = stod(s);

    } catch (std::exception & e) {
        return nullptr;
    }

    Eigen::Vector3d r(rx,ry,rz);
    Eigen::Vector3d t(x,y,z);

    std::array<double, 3> extents{std::abs(dx), std::abs(dy), std::abs(dz)};

    StereoVision::Geometry::RigidBodyTransform<double> rect2world(r,t);
    StereoVision::Geometry::AffineTransform<double> world2rect = rect2world.inverse().toAffineTransform();

    return std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface>(
                new RegionOfInterestSelector(std::move(source), world2rect, extents)
                );
}

RegionOfInterestSelector::RegionOfInterestSelector(std::unique_ptr<StereoVision::IO::PointCloudPointAccessInterface> &&source,
                                                   StereoVision::Geometry::AffineTransform<double> const& transform,
                                                   std::array<double, 3> const& extents) :
    IdentityProcessor(std::move(source)),
    _transform(transform),
    _extends(extents)
{

}

RegionOfInterestSelector::~RegionOfInterestSelector() {

}

bool RegionOfInterestSelector::gotoNext() {

    bool nextIsIn = false;
    bool sourceHasNotEnded = false;

    do {
        sourceHasNotEnded = _src->gotoNext();

        auto pointData = _src->castedPointGeometry<double>();

        Eigen::Vector3d pos;
        pos << pointData.x, pointData.y, pointData.z;

        Eigen::Vector3d transformed = _transform*pos;

        if (std::abs(transformed.x()) <= _extends[0] and
                std::abs(transformed.y()) <= _extends[1] and
                std::abs(transformed.z()) <= _extends[2]) {
            nextIsIn = true;
        }

    } while (!nextIsIn and sourceHasNotEnded);

    return sourceHasNotEnded;
}
