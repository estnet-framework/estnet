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

#include "OffsetMobility.h"

#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/mobility/satellite/SatMobility.h"

namespace estnet {

Define_Module(OffsetMobility);

void OffsetMobility::initialize(int stage)
{
    IExtendedMobility::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        _offset = *new inet::Coord(par("offsetX"), par("offsetY"), par("offsetZ"));
        _orientation = *new inet::Quaternion(inet::EulerAngles((inet::units::values::rad) inet::math::deg2rad(par("yaw")),
                        (inet::units::values::rad) inet::math::deg2rad(par("pitch")),
                        (inet::units::values::rad) inet::math::deg2rad(par("roll"))));

        _nodeMobility = omnetpp::check_and_cast<IExtendedMobility*>(getModuleByPath(par("parentMobility")));
    }
}

inet::Coord OffsetMobility::getCurrentPosition() {
    return this->_nodeMobility->getCurrentPosition() + this->_nodeMobility->getCurrentAngularPosition().rotate(this->_offset);
}

inet::Quaternion OffsetMobility::getCurrentAngularPosition() {
    inet::Quaternion nodeOrientation =
            this->_nodeMobility->getCurrentAngularPosition();
    inet::Quaternion antennaWorldOrientation = nodeOrientation
            * this->_orientation;

    return antennaWorldOrientation;
}

inet::Coord OffsetMobility::getCurrentVelocity() {
    // this mobility is static, so we just have the nodes speed
    return this->_nodeMobility->getCurrentVelocity();
}

inet::Coord OffsetMobility::getCurrentAcceleration() {
    return inet::Coord();
}

inet::Quaternion OffsetMobility::getCurrentAngularVelocity() {
    // this mobility is static, so we just have the nodes speed
    return this->_nodeMobility->getCurrentAngularVelocity();
}

inet::Quaternion OffsetMobility::getCurrentAngularAcceleration() {
    return inet::Quaternion();
}

inet::Coord OffsetMobility::getConstraintAreaMax() const {
    return this->_nodeMobility->getConstraintAreaMax() + this->_offset;
}

inet::Coord OffsetMobility::getConstraintAreaMin() const {
    return this->_nodeMobility->getConstraintAreaMin() + this->_offset;
}

double OffsetMobility::getMaxSpeed() const {
    // this mobility is static, so we just have the nodes speed
    return this->_nodeMobility->getMaxSpeed();
}

inet::Coord OffsetMobility::getPositionAtTime(double time) {
    SatMobility* satMobility;
    try {
        satMobility = dynamic_cast<SatMobility*>(_nodeMobility);
    } catch (const omnetpp::cRuntimeError &e) {
        satMobility = nullptr;
    }
    if (satMobility != nullptr) { // only SatMobility propagates angular position
        return this->_nodeMobility->getPositionAtTime(time) + satMobility->getAngularPositionAtTime(time).rotate(this->_offset);
    } else { //
        return this->_nodeMobility->getPositionAtTime(time) + this->_nodeMobility->getCurrentAngularPosition().rotate(this->_offset);
    }
}
}  // namespace estnet
