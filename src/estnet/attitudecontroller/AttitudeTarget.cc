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

#include "AttitudeTarget.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/terrestrial/StaticTerrestrialMobility.h"

namespace estnet {

inet::Coord AttitudeTarget::_sunPosition;

AttitudeTarget::AttitudeTarget() {
    this->nilTarget = true;
    this->earth_CenterTarget = false;
    this->coordTarget = false;
    this->sunTarget = false;
    this->nodeNumberTarget = false;
    AttitudeTarget::initSunPos();
}

AttitudeTarget::AttitudeTarget(unsigned int target) {
    this->target = target;
    this->nilTarget = false;
    this->earth_CenterTarget = false;
    this->coordTarget = false;
    this->sunTarget = false;
    this->nodeNumberTarget = true;
    AttitudeTarget::initSunPos();
}

AttitudeTarget::AttitudeTarget(std::string target) {
    AttitudeTarget::initSunPos();
    if (target == "NIL") {
        this->nilTarget = true;
        this->earth_CenterTarget = false;
        this->coordTarget = false;
        this->sunTarget = false;
        this->nodeNumberTarget = false;
    } else if (target == "EARTH_CENTER") {
        this->earth_CenterTarget = true;
        this->nilTarget = false;
        this->coordTarget = false;
        this->sunTarget = false;
        this->nodeNumberTarget = false;
    } else if (target == "SUN") {
        this->nilTarget = false;
        this->earth_CenterTarget = false;
        this->coordTarget = false;
        this->sunTarget = true;
        this->nodeNumberTarget = false;
        this->targetCoord = AttitudeTarget::_sunPosition;
    } else {
        this->earth_CenterTarget = false;
        this->nilTarget = false;
        this->coordTarget = false;
        this->sunTarget = false;
        this->nodeNumberTarget = true;
        this->target = stoi(target);
    }
}

AttitudeTarget::AttitudeTarget(inet::Coord target) {
    this->targetCoord = target;
    this->coordTarget = true;
    this->nilTarget = false;
    this->earth_CenterTarget = false;
    this->sunTarget = false;
    this->nodeNumberTarget = false;
    AttitudeTarget::initSunPos();
}

void AttitudeTarget::initSunPos(double sunOrientation) {
    if (AttitudeTarget::_sunPosition.isUnspecified()
            || AttitudeTarget::_sunPosition.isNil()
            || AttitudeTarget::_sunPosition.length() == 0) {
        // calculate Sun Position, setting sun depending on the angle into the x-z-plane of ECI
        AttitudeTarget::_sunPosition = inet::Coord();
        if (std::abs(sunOrientation) > 23.5) {
            throw omnetpp::cRuntimeError(
                    "Attitude Target: invalid sun angle, should be between -23.5 to 23.5 deg");
        }
        sunOrientation = sunOrientation * M_PI / 180;
        double R_SUN = 150000000000.0; //distance sun <-> earth, in m
        AttitudeTarget::_sunPosition.x = R_SUN * cos(sunOrientation);
        AttitudeTarget::_sunPosition.y = 0.0;
        AttitudeTarget::_sunPosition.z = R_SUN * sin(sunOrientation);
    }
}

bool AttitudeTarget::isEarth_Center() const {
    return this->earth_CenterTarget;
}

bool AttitudeTarget::isNil() const {
    return this->nilTarget;
}

bool AttitudeTarget::isSun() const {
    return this->sunTarget;
}

bool AttitudeTarget::isNodeNumber() const {
    return this->nodeNumberTarget;
}

bool AttitudeTarget::isNodeNumber(unsigned int &nodeNo) const {
    nodeNo = this->target;
    return this->nodeNumberTarget;
}

bool AttitudeTarget::isCoord() const {
    return this->coordTarget;
}

void AttitudeTarget::getTargetCoord(inet::Coord &targetCoord) {
    if (isEarth_Center()) {
        targetCoord = inet::Coord::ZERO;
    } else if (isNil()) {
        targetCoord = inet::Coord::NIL;
    } else if (isCoord() || isSun()) {
        targetCoord = this->targetCoord;
    } else {
        NodeBase *node = NodeRegistry::getInstance()->getNode(this->target);
        targetCoord = node->getMobility()->getCurrentPosition();
    }
}

inet::Coord AttitudeTarget::getTargetCoord() {
    if (isEarth_Center()) {
        return inet::Coord::ZERO;
    } else if (isNil()) {
        return inet::Coord::NIL;
    } else if (isCoord() || isSun()) {
        return this->targetCoord;
    } else {
        NodeBase *node = NodeRegistry::getInstance()->getNode(this->target);
        return node->getMobility()->getCurrentPosition();
    }
}

void AttitudeTarget::getTargetCoordAtTime(inet::Coord &targetCoord,
        double time) {
    if (isEarth_Center()) {
        targetCoord = inet::Coord::ZERO;
    } else if (isNil()) {
        targetCoord = inet::Coord::NIL;
    } else if (isCoord() || isSun()) {
        targetCoord = this->targetCoord;
    } else {
        auto mobility =
                NodeRegistry::getInstance()->getNode(this->target)->getMobility();
        auto satMobility = dynamic_cast<SatMobility*>(mobility);
        if (satMobility != nullptr) {
            targetCoord = satMobility->getPositionAtTime(time);
            return;
        }
        auto terMobility = dynamic_cast<StaticTerrestrialMobility*>(mobility);
        if (terMobility != nullptr) {
            targetCoord = terMobility->getPositionAtTime(time);
            return;
        }
        throw omnetpp::cRuntimeError("No matching mobility found.");
    }
}

void AttitudeTarget::getTargetVelocity(inet::Coord &targetVelocity) {
    if (isEarth_Center()) {
        targetVelocity = inet::Coord::ZERO;
    } else if (isNil()) {
        targetVelocity = inet::Coord::NIL;
    } else if (isCoord() || isSun()) {
        targetVelocity = inet::Coord::ZERO;
    } else {
        NodeBase *node = NodeRegistry::getInstance()->getNode(this->target);
        targetVelocity = node->getMobility()->getCurrentVelocity();
    }
}

void AttitudeTarget::getTargetVelocityAtTime(inet::Coord &targetVelocity,
        double time) {
    if (isEarth_Center()) {
        targetVelocity = inet::Coord::ZERO;
    } else if (isNil()) {
        targetVelocity = inet::Coord::NIL;
    } else if (isCoord() || isSun()) {
        targetVelocity = inet::Coord::ZERO;
    } else {
        auto mobility =
                NodeRegistry::getInstance()->getNode(this->target)->getMobility();
        auto satMobility = dynamic_cast<SatMobility*>(mobility);
        if (satMobility != nullptr) {
            targetVelocity = satMobility->getVelocityAtTime(time);
            return;
        }
        auto terMobility = dynamic_cast<StaticTerrestrialMobility*>(mobility);
        if (terMobility != nullptr) {
            targetVelocity = terMobility->getVelocityAtTime(time);
            return;
        }
        throw omnetpp::cRuntimeError("No matching mobility found.");
    }
}

}  // namespace estnet
