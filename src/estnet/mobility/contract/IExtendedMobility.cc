//
// Copyright (C) 2020 Computer Science VII: Robotics and Telematics - 
// Julius-Maximilians-Universitaet Wuerzburg
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "IExtendedMobility.h"

#include "estnet/environment/earthmodel/EarthModelFactory.h"
#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/mobility/satellite/common/QuaternionHelpers.h"

// TODO should not rely on osg matrix in here
// TODO to much copying around between matrix arrays and matrix classes

namespace estnet {

IExtendedMobility::IExtendedMobility() {
    // osgearth uses WGS84 by default to calculate the ENU coordinate system, so
    // we'll do the same
    this->_osgEarthModel = EarthModelFactory::get(
            EarthModelFactory::EarthModels::WGS84);
    this->_jdGlobal = GlobalJulianDate::getInstancePtr();
}

IExtendedMobility::~IExtendedMobility() {
    delete this->_osgEarthModel;
}

M4x4d IExtendedMobility::getCurrentAngularPositionRelativeToENU(cJulian time,
        const inet::Coord &pos) {
    // we compute the matrix that reverses OSGs ENU coordinate frame to the world
    // corrdinate frame
    double reverseEnuMatPtr[3][3];
    this->getEnuToWorldMatrix(time, pos, reverseEnuMatPtr);
    M4x4d reverseEnuMat;
    reverseEnuMat << reverseEnuMatPtr[0][0], reverseEnuMatPtr[0][1], reverseEnuMatPtr[0][2], 0.0, reverseEnuMatPtr[1][0], reverseEnuMatPtr[1][1], reverseEnuMatPtr[1][2], 0.0, reverseEnuMatPtr[2][0], reverseEnuMatPtr[2][1], reverseEnuMatPtr[2][2], 0.0, 0.0, 0.0, 0.0, 1.0;

    double attitudeMatPtr[3][3];
    quaternionToMatrix(this->getCurrentAngularPosition(), attitudeMatPtr);

    M4x4d attitudeMat;
    attitudeMat << attitudeMatPtr[0][0], attitudeMatPtr[0][1], attitudeMatPtr[0][2], 0.0, attitudeMatPtr[1][0], attitudeMatPtr[1][1], attitudeMatPtr[1][2], 0.0, attitudeMatPtr[2][0], attitudeMatPtr[2][1], attitudeMatPtr[2][2], 0.0, 0.0, 0.0, 0.0, 1.0;

    return (reverseEnuMat * attitudeMat).transpose(); // simulation attitude
}

void IExtendedMobility::getEnuToWorldMatrix(cJulian time, const inet::Coord &p,
        double matrix[3][3]) {

    // we're reversing the ENU coordinate system osgEarth forces onto nodes in
    // geo-centric maps.
    // For reference in osg-3.5.7-d649334 & osgEarth-2.9.0-d5ae389:
    // - osgEarth src/osgEarthFeatures/SubstitudeModelFilter.cpp:128 SubstituteModelFilter::process
    //   makeEcef variable is true if map is geocentric
    // - osgEarth src/osgEarthFeatures/SubstitudeModelFilter.cpp:313 SubstituteModelFilter::process
    //   rotation from ECEF::transformAndGetRotationMatrix is added into rotation matrix of node
    // - osgEarth src/osgEarth/ECEF.cpp:173 ECEF::transformAndGetRotationMatrix
    //   calls EllipsoidModel::computeCoordinateFrame
    // - osg include/osg/CoordinateSystemNode:208 EllipsoidModel::computeCoordinateFrame
    //   calculates ENU coordinate frame based on lat/lon

    inet::Coord east, north, up;
    inet::Coord x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
    this->_osgEarthModel->computeNadirCoordinateFrameFromECI(time, p, east,
            north, up);

    // we're saving the transposed matrix, which inverts the rotation into the ENU
    // coordinate system
    matrix[0][0] = east.x;
    matrix[0][1] = east.y;
    matrix[0][2] = east.z;
    matrix[1][0] = north.x;
    matrix[1][1] = north.y;
    matrix[1][2] = north.z;
    matrix[2][0] = up.x;
    matrix[2][1] = up.y;
    matrix[2][2] = up.z;
}

void IExtendedMobility::getWorldToEnuMatrix(cJulian time, const inet::Coord &p,
        double matrix[3][3]) {
    double enuToWorld[3][3];
    this->getEnuToWorldMatrix(time, p, enuToWorld);

    // transposing to get inverse
    matrix[0][0] = enuToWorld[0][0];
    matrix[0][1] = enuToWorld[1][0];
    matrix[0][2] = enuToWorld[2][0];
    matrix[1][0] = enuToWorld[0][1];
    matrix[1][1] = enuToWorld[1][1];
    matrix[1][2] = enuToWorld[2][1];
    matrix[2][0] = enuToWorld[0][2];
    matrix[2][1] = enuToWorld[1][2];
    matrix[2][2] = enuToWorld[2][2];
}

inet::EulerAngles IExtendedMobility::getWorldToEnuAngles(cJulian time,
        const inet::Coord &p) {
    double yaw = 0, pitch = 0, roll = 0;

    double R[3][3];
    this->getWorldToEnuMatrix(time, p, R);

    // grab the euler angles the rotation matrix represents
    getEulerAnglesFromMatrix(R, yaw, pitch, roll);

    // get latitude and longitude for special cases
    inetu::deg latitude, longitude;
    inetu::m altitude;
    this->_osgEarthModel->convertECIToLatLongHeight(time, p, latitude,
            longitude, altitude);
    // at the poles ENU can be arbitrarily rotated around the z-axis
    // osgEarth doesn't align it with any axis of the world frame, so
    // we're gonna make these two special cases and define the unique
    // coordinate frame here
    if (std::fabs(latitude.get() - 90) < 0.01) {
        yaw = 0;
        pitch = 0;
        roll = 0;
    } else if (std::fabs(latitude.get() + 90) < 0.01) {
        yaw = 0;
        pitch = M_PI;
        roll = 0;
    }

    return inet::EulerAngles((inet::units::values::rad) yaw,
            (inet::units::values::rad) pitch, (inet::units::values::rad) roll);
}

}  // namespace estnet
