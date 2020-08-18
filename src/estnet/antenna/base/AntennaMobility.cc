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

#include "AntennaMobility.h"

#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"

namespace estnet {

AntennaMobility::AntennaMobility(inet::IMobility *nodeMobility, double yaw,
        double pitch, double roll) :
        _nodeMobility(nodeMobility), _offset(0, 0, 0), _orientation(
                inet::EulerAngles(
                        (inet::units::values::rad) inet::math::deg2rad(yaw),
                        (inet::units::values::rad) inet::math::deg2rad(pitch),
                        (inet::units::values::rad) inet::math::deg2rad(roll))) {

}

inet::Coord AntennaMobility::getCurrentPosition() {
    return this->_nodeMobility->getCurrentPosition() + this->_offset;
}

inet::Quaternion AntennaMobility::getCurrentAngularPosition() {
    inet::Quaternion nodeOrientation =
            this->_nodeMobility->getCurrentAngularPosition();
    inet::Quaternion antennaWorldOrientation = nodeOrientation
            * this->_orientation;

    return antennaWorldOrientation;
}

inet::Coord AntennaMobility::getCurrentVelocity() {
    // this mobility is static, so we just have the nodes speed
    return this->_nodeMobility->getCurrentVelocity();
}

inet::Coord AntennaMobility::getCurrentAcceleration() {
    return inet::Coord();
}

inet::Quaternion AntennaMobility::getCurrentAngularVelocity() {
    // this mobility is static, so we just have the nodes speed
    return this->_nodeMobility->getCurrentAngularVelocity();
}

inet::Quaternion AntennaMobility::getCurrentAngularAcceleration() {
    return inet::Quaternion();
}

inet::Coord AntennaMobility::getConstraintAreaMax() const {
    return this->_nodeMobility->getConstraintAreaMax() + this->_offset;
}

inet::Coord AntennaMobility::getConstraintAreaMin() const {
    return this->_nodeMobility->getConstraintAreaMin() + this->_offset;
}

double AntennaMobility::getMaxSpeed() const {
    // this mobility is static, so we just have the nodes speed
    return this->_nodeMobility->getMaxSpeed();
}

inet::Coord AntennaMobility::getPositionAtTime(double time) {
    auto extMobility = omnetpp::check_and_cast<IExtendedMobility*>(
            this->_nodeMobility);
    // FIXME: has this to be a coordinate frame transformation as offset is in satellite frame
    return extMobility->getPositionAtTime(time) + this->_offset;
}
}  // namespace estnet
