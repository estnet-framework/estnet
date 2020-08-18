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

#include "PropStateEulerAngles.h"

namespace estnet {

PropStateEulerAngles::PropStateEulerAngles() :
        PropStateAttitude(), _attitude(), _angularVelocity(), _angularAcceleration() {
}

void PropStateEulerAngles::attitudeAsEulerAngles(cEulerAngles &eAngles) const {
    eAngles = this->_attitude;
}

void PropStateEulerAngles::attitudeAsQuaternion(cQuaternion &quaternion) const {
    quaternion = *new cQuaternion(this->_attitude);
}

void PropStateEulerAngles::angularVelAsEulerAngles(
cEulerAngles &eAngles) const {
    eAngles = this->_angularVelocity;
}

void PropStateEulerAngles::angularVelAsQuaternion(
cQuaternion &quaternion) const {
    quaternion = *new cQuaternion(this->_angularVelocity);
}

void PropStateEulerAngles::angularAccAsEulerAngles(
cEulerAngles &eAngles) const {
    eAngles = this->_angularAcceleration;
}

void PropStateEulerAngles::angularAccAsQuaternion(
cQuaternion &quaternion) const {
    quaternion = *new cQuaternion(this->_angularAcceleration);
}

void PropStateEulerAngles::attitudeFromEulerAngles(
        const cEulerAngles &eAngles) {
    this->_attitude = eAngles;
}

void PropStateEulerAngles::attitudeFromQuaternion(
        const cQuaternion &quaternion) {
    this->_attitude = quaternion.toEulerAngles(true);
}

void PropStateEulerAngles::angularVelFromEulerAngles(
        const cEulerAngles &eAngles) {
    this->_angularVelocity = eAngles;
}

void PropStateEulerAngles::angularVelFromQuaternion(
        const cQuaternion &quaternion) {
    this->_angularVelocity = quaternion.toEulerAngles(true);
}

void PropStateEulerAngles::angularAccFromEulerAngles(
        const cEulerAngles &eAngles) {
    this->_angularAcceleration = eAngles;
}

void PropStateEulerAngles::angularAccFromQuaternion(
        const cQuaternion &quaternion) {
    this->_angularAcceleration = quaternion.toEulerAngles(true);
}

}  // namespace estnet
