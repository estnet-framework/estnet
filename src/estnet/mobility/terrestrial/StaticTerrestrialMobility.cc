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

#include <iostream>

#include "inet/common/geometry/common/RotationMatrix.h"

#include "estnet/environment/earthmodel/EarthModelFactory.h"
#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/node/tracking/contract/INodeTracking.h"


namespace estnet {

Define_Module(StaticTerrestrialMobility)

omnetpp::simsignal_t StaticTerrestrialMobility::az = registerSignal("azimuth");
omnetpp::simsignal_t StaticTerrestrialMobility::el = registerSignal(
        "elevation");
omnetpp::simsignal_t StaticTerrestrialMobility::azEr = registerSignal(
        "azimuthError");
omnetpp::simsignal_t StaticTerrestrialMobility::elEr = registerSignal(
        "elevationError");
omnetpp::simsignal_t StaticTerrestrialMobility::positionUpdateX =
  registerSignal("positionUpdateX");
omnetpp::simsignal_t StaticTerrestrialMobility::positionUpdateY =
  registerSignal("positionUpdateY");
omnetpp::simsignal_t StaticTerrestrialMobility::positionUpdateZ =
  registerSignal("positionUpdateZ");
omnetpp::simsignal_t StaticTerrestrialMobility::velocityUpdateX =
  registerSignal("velocityUpdateX");
omnetpp::simsignal_t StaticTerrestrialMobility::velocityUpdateY =
  registerSignal("velocityUpdateY");
omnetpp::simsignal_t StaticTerrestrialMobility::velocityUpdateZ =
  registerSignal("velocityUpdateZ");
  
void StaticTerrestrialMobility::initialize(int stage) {
    IExtendedMobility::initialize(stage);

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

void StaticTerrestrialMobility::getCurrentOrientationAngles(double &azimuth, double &elevation) {
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

    // Converts the current orientation from ECI to ENU
    inet::Quaternion orientENU = q * _orientation;

    inet::Coord pointing = orientENU.rotate(inet::Coord(1, 0, 0));

    if (pointing.z > 1.0)
        pointing.z = 1.0;
    else if (pointing.z < -1.0)
        pointing.z = -1.0;

    // converts the rotation to azimuth and elevation
    elevation = asin(pointing.z);
    azimuth = -atan2(-pointing.x, pointing.y);
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

double StaticTerrestrialMobility::getMaxSpeed() const {
    // return maximum velocity possible for the given height
    double r = alt.get() + WGS_84_RADIUS_EQUATOR;
    double omega = EARTH_SPIN * M_PI / 180;
    return r * omega;
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

    this->emit(positionUpdateX, pos.x);
    this->emit(positionUpdateY, pos.y);
    this->emit(positionUpdateZ, pos.z);

    return pos;
}

inet::Coord StaticTerrestrialMobility::getCurrentVelocity() {
    inet::Coord omega = inet::Coord(0, 0, EARTH_SPIN * M_PI / 180);
    inet::Coord vel = omega % this->getCurrentPosition();
    this->emit(velocityUpdateX, vel.x);
    this->emit(velocityUpdateY, vel.y);
    this->emit(velocityUpdateZ, vel.z);
    return vel;
}

inet::Coord StaticTerrestrialMobility::getVelocityAtTime(double time) {
    inet::Coord omega = inet::Coord(0, 0, EARTH_SPIN * M_PI / 180);

    return omega % this->getPositionAtTime(time);
}

inet::Coord StaticTerrestrialMobility::getCurrentAcceleration() {
    double v = this->getCurrentVelocity().length();
    inet::Coord r = this->getCurrentPosition();
    double omega = v/r.length();
    return r * ((float)-pow(omega, 2));
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
    // TODO
    throw omnetpp::cRuntimeError("Computing the angular velocity of a ground station is currently not supported. "
            "All calculations will be incorrect, that rely on this.");
    return inet::Quaternion::NIL;
}

inet::Quaternion StaticTerrestrialMobility::getCurrentAngularAcceleration() {
    // TODO
    throw omnetpp::cRuntimeError("Computing the angular acceleration of a ground station is currently not supported. "
                "All calculations will be incorrect, that rely on this.");
    return inet::Quaternion::NIL;
}

inet::Coord StaticTerrestrialMobility::getConstraintAreaMax() const {
    double height = alt.get() + WGS_84_RADIUS_EQUATOR;
    return inet::Coord(height, height, height);
}

inet::Coord StaticTerrestrialMobility::getConstraintAreaMin() const {
    return inet::Coord(0, 0, 0);
}

}  // namespace estnet
