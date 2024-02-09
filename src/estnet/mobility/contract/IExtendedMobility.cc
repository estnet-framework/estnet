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


namespace estnet {

IExtendedMobility::IExtendedMobility(): _doAutoUpdate(
        false), _selfUpdateIV_s(0.0){
    // osgearth uses WGS84 by default to calculate the ENU coordinate system, so
    // we'll do the same
    this->_osgEarthModel = EarthModelFactory::get(
            EarthModelFactory::EarthModels::WGS84);
    this->_jdGlobal = GlobalJulianDate::getInstancePtr();
}

IExtendedMobility::~IExtendedMobility() {
    delete this->_osgEarthModel;
}

void IExtendedMobility::handleMessage(omnetpp::cMessage *msg) {
    if (msg == this->_updateTimer) {
        this->getCurrentPosition();
        this->getCurrentVelocity();
        this->scheduleAt(omnetpp::simTime() + this->_selfUpdateIV_s,
                this->_updateTimer);
    }
}

void IExtendedMobility::initialize(int stage) {
    // get parameters
    if (stage == inet::INITSTAGE_LOCAL) {
        _doAutoUpdate = par("enableSelfTrigger");
         _selfUpdateIV_s = par("selfTriggerTimeIv").doubleValueInUnit("s");
         if(_doAutoUpdate){
             _updateTimer = new omnetpp::cMessage("StateUpdateTimer");
             this->scheduleAt(this->_selfUpdateIV_s, this->_updateTimer);
         }
    }
}
M4x4d IExtendedMobility::getCurrentAngularPositionRelativeToENU(cJulian time,
        const inet::Coord &pos) {
    // we compute the matrix that reverses OSGs ENU coordinate frame to the world
    // corrdinate frame
    M4x4d reverseEnuMat;
    this->getEnuToWorldMatrix(time, pos, reverseEnuMat);

    M4x4d attitudeMat;
    quaternionToMatrix(this->getCurrentAngularPosition(), attitudeMat);

    return (reverseEnuMat * attitudeMat).transpose(); // simulation attitude
}

void IExtendedMobility::getEnuToWorldMatrix(cJulian time, const inet::Coord &p,
        M4x4d& matrix) {

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

    this->_osgEarthModel->computeNadirCoordinateFrameFromECI(time, p, east,
            north, up);

    // we're saving the transposed matrix, which inverts the rotation into the ENU
    // coordinate system
    matrix << east.x, east.y, east.z, 0.0, 
        north.x, north.y, north.z, 0.0, 
        up.x, up.y, up.z, 0.0, 
        0.0, 0.0, 0.0, 1.0;
}

void IExtendedMobility::getWorldToEnuMatrix(cJulian time, const inet::Coord &p,
        M4x4d& matrix) {
    this->getEnuToWorldMatrix(time, p, matrix);

    // transposing to get inverse
    matrix.transpose();
}

inet::EulerAngles IExtendedMobility::getWorldToEnuAngles(cJulian time,
        const inet::Coord &p) {
    double yaw = 0, pitch = 0, roll = 0;

    M4x4d R;
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
