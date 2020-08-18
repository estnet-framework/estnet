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

#include "WeightedSwipeTracking.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/contactplan/common/ContactPlanManager.h"

namespace estnet {

Define_Module(WeightedSwipeTracking);

void WeightedSwipeTracking::reschedule(std::map<unsigned int, double> schedule,
        double ratioOfPassTime) {
    this->_scheduled = true;
    this->_schedule = schedule;
    this->_ratioOfPassTime = ratioOfPassTime;
    if (this->isTracking()) {
        updateTimeParametersRescheduled(true);
    }
}

void WeightedSwipeTracking::handleMessage(cMessage *message) {

    ISwipeTracking::handleMessage(message);
    if (message == this->_swipeStart) {
        if (this->_scheduled) {
            updateTimeParametersRescheduled(true);
            this->_scheduled = false;
        }
    } else if (message == this->_endOfReschedule) {
        updateTimeParametersRescheduled(false);

    }

}

inet::Quaternion WeightedSwipeTracking::getNewOrientation() {
    auto t = GlobalJulianDate::getInstance().currentSimTime();
    double ratioOfPassedOverflyTime = t.spanSec(_T.first) / _trackingTime;
    Satellite *sat;
    double tSim;
    size_t i;
    int closestSat;
    if (_trackingNodeIds.second > _trackingNodeIds.first) {
        double sum = 0.0;
        for (i = _trackingNodeIds.first; i <= _trackingNodeIds.second; i++) {
            sum += _schedule.at(i);
            if (sum >= ratioOfPassedOverflyTime) {
                break;
            }
        }
        closestSat = i
                + std::min(_trackingNodeIds.second, _trackingNodeIds.first);
    } else {
        double sum = 0.0;
        for (i = _trackingNodeIds.first; i >= _trackingNodeIds.second; i--) {
            sum += _schedule.at(i);
            if (sum >= ratioOfPassedOverflyTime) {
                break;
            }
        }
        closestSat = -i
                + std::max(_trackingNodeIds.second, _trackingNodeIds.first);
    }
    //ratioOfPassedOverflyTime = numSats * ratioOfPassedOverflyTime - (double)ratioInSatellites;
    // change satellite on which calculation is based
    //t.addSec(ratioOfPassedOverflyTime * _deltaT);
    tSim = GlobalJulianDate::getInstance().julianDate2SimTime(t);
    sat = NodeRegistry::getInstance()->getSatellite(closestSat);

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

void WeightedSwipeTracking::updateTimeParametersRescheduled(
        bool startReschedule) {
    if (startReschedule) {
        _trackingTime *= _ratioOfPassTime;

        cJulian newEndTime = cJulian(_T.first);
        newEndTime.addSec(_trackingTime);
        _T.second = newEndTime;
        this->scheduleAt(
                GlobalJulianDate::getInstance().julianDate2SimTime(_T.second),
                this->_endOfReschedule);

    } else {
        // calculate remaining time
        double rescheduledTime = _trackingTime;
        _trackingTime /= _ratioOfPassTime;
        _trackingTime -= rescheduledTime;

        // set start point that is the end point of reschedule
        _T.first = _T.second;
        cJulian newEndTime = cJulian(_T.first);
        newEndTime.addSec(_trackingTime);
        _T.second = newEndTime;
    }
}

} //namespace
