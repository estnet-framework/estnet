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

#include "StaticTerrestrialMobility.h"

#include "estnet/environment/earthmodel/EarthModelFactory.h"
#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/node/tracking/contract/INodeTracking.h"
#include "inet/common/geometry/common/RotationMatrix.h"

#include <iostream>

namespace estnet {

Define_Module(StaticTerrestrialMobility)

omnetpp::simsignal_t StaticTerrestrialMobility::az = registerSignal("azimuth");
omnetpp::simsignal_t StaticTerrestrialMobility::el = registerSignal(
        "elevation");
omnetpp::simsignal_t StaticTerrestrialMobility::azEr = registerSignal(
        "azimuthError");
omnetpp::simsignal_t StaticTerrestrialMobility::elEr = registerSignal(
        "elevationError");

void StaticTerrestrialMobility::initialize(int stage) {
    if (stage == inet::INITSTAGE_LOCAL) {
        _orientation = inet::Quaternion::IDENTITY;
        // the omnet obstacle is a sphere, so we're gonna use a sphere for ground
        // station positions as well
        _earthModel = EarthModelFactory::get(
                EarthModelFactory::EarthModels::SPHERE);

        // get parameters
        lon = inetu::deg(par("lon").doubleValueInUnit("deg"));
        lat = inetu::deg(par("lat").doubleValueInUnit("deg"));
        alt = inetu::m(par("alt").doubleValueInUnit("m"));

        enableKinematics = par("enableKinematics").boolValue();

        maxAzRate = par("maxAzimuthRate").doubleValue();
        maxElRate = par("maxElevationRate").doubleValue();

        this->_lastUpdateTime =
                GlobalJulianDate::getInstance().currentSimTime();
    }
}

StaticTerrestrialMobility::~StaticTerrestrialMobility() {
    delete this->_earthModel;
}

void StaticTerrestrialMobility::setCurrentAngularPosition(
        const inet::Quaternion &newOrientation) {
    // Constructs an East-Nord-Up (ENU) Cartesian coordinate system at the position of the ground station
    inet::Coord east, north, up;
    this->_earthModel->computeNadirCoordinateFrameFromECI(
            GlobalJulianDate::getInstance().currentSimTime(),
            getCurrentPosition(), east, north, up,
            estnet::reference_system::ECI);
    east.normalize();
    north.normalize();
    up.normalize();
    double R[3][3] = { { east.x, east.y, east.z },
            { north.x, north.y, north.z }, { up.x, up.y, up.z } };
    inet::RotationMatrix rotM(R);
    inet::Quaternion q = rotM.toQuaternion();

    // Converts the old and new orientation from ECI to ENU
    inet::Quaternion orientENU = q * _orientation;
    inet::Quaternion newOrientENU = q * newOrientation;

    inet::Coord oldPointing = orientENU.rotate(inet::Coord(1, 0, 0));
    inet::Coord newPointing = newOrientENU.rotate(inet::Coord(1, 0, 0));

    // converts the rotation to azimuth and elevation
    double oldElevation = asin(oldPointing.z);
    double oldAzimuth = -atan2(-oldPointing.x, oldPointing.y);
    double newElevation = asin(newPointing.z);
    double newAzimuth = -atan2(-newPointing.x, newPointing.y);
    double azChange = newAzimuth - oldAzimuth;
    double elChange = newElevation - oldElevation;
    double azError = 0.0;
    double elError = 0.0;

    // if kinematics are enabled, the new orientation is limited by the turning rate per axis
    if (enableKinematics) {
        double timeDifference =
                GlobalJulianDate::getInstance().currentSimTime().spanSec(
                        this->_lastUpdateTime);
        this->_lastUpdateTime =
                GlobalJulianDate::getInstance().currentSimTime();

        if (fabs(azChange) > timeDifference * maxAzRate) {
            EV_WARN
                           << "Groundstation exceeded maximum angular rate for the azimuth angle!"
                           << endl;
            azError = azChange
                    - timeDifference * maxAzRate * copysign(1.0, azChange);
            newAzimuth = oldAzimuth
                    + timeDifference * maxAzRate * copysign(1.0, azChange);
        }
        if (fabs(elChange) > timeDifference * maxElRate) {
            EV_WARN
                           << "Groundstation exceeded maximum angular rate for the elevation angle!"
                           << endl;
            elError = elChange
                    - timeDifference * maxElRate * copysign(1.0, elChange);
            newElevation = oldElevation
                    + timeDifference * maxElRate * copysign(1.0, elChange);
        }

        inet::Coord newVec = inet::Coord(cos(newElevation) * sin(newAzimuth),
                cos(newElevation) * cos(newAzimuth), sin(newElevation));
        inet::Quaternion qConj = q;
        qConj.conjugate();
        inet::Coord newVecECI = qConj.rotate(newVec);

        _orientation = inet::Quaternion::rotationFromTo(inet::Coord(1, 0, 0),
                newVecECI);
    } else {
        _orientation = newOrientation;
    }
    this->emit(az, fmod(360 + newAzimuth * DEGS_PER_RAD, 360));
    this->emit(el, newElevation * DEGS_PER_RAD);
    this->emit(azEr, azError * DEGS_PER_RAD);
    this->emit(elEr, elError * DEGS_PER_RAD);

}

void StaticTerrestrialMobility::handleMessage(omnetpp::cMessage *msg) {
}

double StaticTerrestrialMobility::getMaxSpeed() const {
    // TODO: Max speed calculation...
    return 0.0;
}

inet::Coord StaticTerrestrialMobility::getPositionAtTime(double time) {
    inet::Coord pos;
    cJulian simTime = _jdGlobal->simTime2JulianDate(time);
    _earthModel->convertLatLongHeightToECI(simTime, lat, lon, alt, pos);
    return pos;
}

inet::Coord StaticTerrestrialMobility::getCurrentPosition() {
    inet::Coord pos;
    cJulian simTime = _jdGlobal->currentSimTime();
    _earthModel->convertLatLongHeightToECI(simTime, lat, lon, alt, pos);
    return pos;
}

inet::Coord StaticTerrestrialMobility::getCurrentVelocity() {
    // TODO
    return inet::Coord();
}

inet::Coord StaticTerrestrialMobility::getVelocityAtTime(double time) {
    // TODO
    return inet::Coord();
}

inet::Coord StaticTerrestrialMobility::getCurrentAcceleration() {
    // TODO
    return inet::Coord();
}

inet::Quaternion StaticTerrestrialMobility::getCurrentAngularPosition() {
    auto gsModule = this->getParentModule()->getParentModule();
    auto trackingModule = gsModule->getSubmodule("nodeTracking");
    if (trackingModule != nullptr) {
        auto tracking = check_and_cast<INodeTracking*>(trackingModule);
        tracking->updateAttitude();
    }
    return _orientation;
}

inet::Quaternion StaticTerrestrialMobility::getCurrentAngularVelocity() {
    // TODO: For Groundstations everytime in Satellite-Direction
    return inet::Quaternion::NIL;
}

inet::Quaternion StaticTerrestrialMobility::getCurrentAngularAcceleration() {
    // TODO
    return inet::Quaternion::NIL;
}

inet::Coord StaticTerrestrialMobility::getConstraintAreaMax() const {
    // TODO: return true constraint area max
    return inet::Coord();
}

inet::Coord StaticTerrestrialMobility::getConstraintAreaMin() const {
    // TODO: return true constraint area min
    return inet::Coord();
}

}  // namespace estnet
