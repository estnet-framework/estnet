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

#include "PropStateQuaternion.h"

namespace estnet {

PropStateQuaternion::PropStateQuaternion() :
        PropStateAttitude(), _attitude(), _angularVelocity(), _angularAcceleration() {
}

void PropStateQuaternion::attitudeAsEulerAngles(cEulerAngles &eAngles) const {
    cEulerAngles tmp = this->_attitude.toEulerAngles(true);
    eAngles.alpha = tmp.alpha;
    eAngles.beta = tmp.beta;
    eAngles.gamma = tmp.gamma;
}

void PropStateQuaternion::attitudeAsQuaternion(cQuaternion &quaternion) const {
    quaternion = this->_attitude;
}

void PropStateQuaternion::angularVelAsEulerAngles(cEulerAngles &eAngles) const {
    cEulerAngles tmp = this->_angularVelocity.toEulerAngles(true);
    eAngles.alpha = tmp.alpha;
    eAngles.beta = tmp.beta;
    eAngles.gamma = tmp.gamma;
}

void PropStateQuaternion::angularVelAsQuaternion(
cQuaternion &quaternion) const {
    quaternion = this->_angularVelocity;
}

void PropStateQuaternion::angularAccAsEulerAngles(cEulerAngles &eAngles) const {
    cEulerAngles tmp = this->_angularAcceleration.toEulerAngles(true);
    eAngles.alpha = tmp.alpha;
    eAngles.beta = tmp.beta;
    eAngles.gamma = tmp.gamma;
}

void PropStateQuaternion::angularAccAsQuaternion(
cQuaternion &quaternion) const {
    quaternion = this->_angularAcceleration;
}

void PropStateQuaternion::attitudeFromEulerAngles(const cEulerAngles &eAngles) {
    this->_attitude = cQuaternion(eAngles);
}

void PropStateQuaternion::attitudeFromQuaternion(
        const cQuaternion &quaternion) {
    this->_attitude = quaternion;
}

void PropStateQuaternion::angularVelFromEulerAngles(
        const cEulerAngles &eAngles) {
    this->_angularVelocity = cQuaternion(eAngles);
    this->_angularVelocity.setS(0.0);
}

void PropStateQuaternion::angularVelFromQuaternion(
        const cQuaternion &quaternion) {
    this->_angularVelocity = quaternion;
    this->_angularVelocity.setS(0.0);
}

void PropStateQuaternion::angularAccFromEulerAngles(
        const cEulerAngles &eAngles) {
    this->_angularAcceleration = cQuaternion(eAngles);
    this->_angularAcceleration.setS(0.0);
}

void PropStateQuaternion::angularAccFromQuaternion(
        const cQuaternion &quaternion) {
    this->_angularAcceleration = quaternion;
    this->_angularAcceleration.setS(0.0);
}

}  // namespace estnet
