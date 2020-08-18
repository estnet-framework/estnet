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

#include "ConstantSwipeTracking.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/contactplan/common/ContactPlanManager.h"

namespace estnet {

Define_Module(ConstantSwipeTracking);

inet::Quaternion ConstantSwipeTracking::getNewOrientation() {
    // get current simulation time
    auto t = GlobalJulianDate::getInstance().currentSimTime();
    // calculate the percentage of current pass
    double ratioOfPassedPassTime = t.spanSec(_T.first) / _trackingTime;

    // get satellite of the id in the center of the formation
    Satellite *sat;
    double tSim;
    double numSats = (double) abs(
            (int) _trackingNodeIds.second - (int) _trackingNodeIds.first) + 1;
    // check for swipe direction
    if (_trackingNodeIds.second > _trackingNodeIds.first) {
        sat = NodeRegistry::getInstance()->getSatellite(
                (int) (_trackingNodeIds.first + numSats / 2));

    } else {
        sat = NodeRegistry::getInstance()->getSatellite(
                (int) (_trackingNodeIds.first - numSats / 2));

    }

    // statistics
    emit(relTarget,
            ratioOfPassedPassTime * (numSats - 1)
                    + (int) _trackingNodeIds.first);

    //shift time
    double offset = 0;
    if (((int) numSats % 2) == 0) {
        offset = 0.5 * _deltaT / (numSats - 1);
    }
    //t.addSec((-(ratioOfPassedPassTime - (double)sat->getNodeNo() / numSats)) * _deltaT/(numSats-1));
    t.addSec(-(ratioOfPassedPassTime - 0.5) * _deltaT + offset);

    tSim = GlobalJulianDate::getInstance().julianDate2SimTime(t);

    if (sat == nullptr) {
        throw omnetpp::cRuntimeError(
                "Tracking something that is not a satellite");
    }
    SatMobility *mobility = dynamic_cast<SatMobility*>(sat->getMobility());
    if (mobility == nullptr) {
        throw omnetpp::cRuntimeError(
                "Tracking something that is not SatMobility");
    }
    inet::Coord targetPositionECI = mobility->getPositionAtTime(tSim);

    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    StaticTerrestrialMobility *m = groundstation->getMobility();
    inet::Coord ourPositionECI = m->getCurrentPosition();

    inet::Quaternion newOrientation = inet::Quaternion::rotationFromTo(
            inet::Coord(1, 0, 0), targetPositionECI - ourPositionECI);
    return newOrientation;
}

} //namespace
