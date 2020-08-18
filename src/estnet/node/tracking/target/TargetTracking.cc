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

#include "TargetTracking.h"

namespace estnet {

Define_Module(TargetTracking);

omnetpp::simsignal_t TargetTracking::targetStats = registerSignal(
        "trackedTarget");

void TargetTracking::initialize(int stage) {
    if (stage == inet::INITSTAGE_LOCAL) {
        INodeTracking::initialize();
        this->_target = *new AttitudeTarget(
                this->par("target").stdstringValue());
        unsigned int nodeNumber;
        if (_target.isNodeNumber(nodeNumber)) {
            emit(targetStats, nodeNumber);
        }
    } else if (stage == inet::INITSTAGE_LAST) {
        start();
    }
}

inet::Quaternion TargetTracking::getNewOrientation() {
    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    StaticTerrestrialMobility *m = groundstation->getMobility();
    if (_target.isNil()) {
        return inet::Quaternion::rotationFromTo(inet::Coord(1, 0, 0),
                m->getCurrentPosition());
    } else {

        return inet::Quaternion::rotationFromTo(inet::Coord(1, 0, 0),
                _target.getTargetCoord() - m->getCurrentPosition());
    }

}

void TargetTracking::setTarget(AttitudeTarget target) {
    _target = target;
    unsigned int nodeNumber;
    if (target.isNodeNumber(nodeNumber)) {
        emit(targetStats, nodeNumber);
    }
}

} //namespace
