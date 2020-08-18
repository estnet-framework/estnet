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

#include "MeanTracking.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/contactplan/common/ContactPlanManager.h"

namespace estnet {

Define_Module(MeanTracking);

omnetpp::simsignal_t MeanTracking::relTarget = registerSignal("relTarget");

inet::Quaternion MeanTracking::getNewOrientation() {
    auto t = SIMTIME_DBL(omnetpp::simTime());

    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    StaticTerrestrialMobility *m = groundstation->getMobility();
    inet::Coord ourPositionECI = m->getCurrentPosition();

    inet::Coord weightedSumOfSatPosition;
    double weightedSumOfSatIds = 0;
    double sumWeight = 0;
    int dsats = (int) _trackingNodeIds.second - (int) _trackingNodeIds.first;
    int numSats = abs(dsats) + 1;

    // iterate over selected satellites
    int increment = dsats / abs(dsats);

    for (int i = (int) _trackingNodeIds.first;
            i <= (int) _trackingNodeIds.second; i += increment) {
        // check if satellite is available
        auto cpManager = ContactPlanManager::getInstance();
        if (!cpManager->hasActiveContactFor(groundstation->getNodeNo(), i)) {
            continue;
        }

        auto sat = NodeRegistry::getInstance()->getSatellite(i);
        if (sat == nullptr) {
            throw omnetpp::cRuntimeError(
                    "Tracking something that is not a satellite");
        }
        SatMobility *mobility = dynamic_cast<SatMobility*>(sat->getMobility());
        if (mobility == nullptr) {
            throw omnetpp::cRuntimeError(
                    "Tracking something that is not SatMobility");
        }
        inet::Coord satPositionECI = mobility->getPositionAtTime(t);
        inet::Coord satPositionLocal = satPositionECI - ourPositionECI;
        double weight = pow(10, _exponent)
                / (pow(satPositionLocal.length(), _exponent) * numSats);
        satPositionLocal *= weight / satPositionLocal.length();
        weightedSumOfSatPosition += satPositionLocal;
        sumWeight += weight;
        weightedSumOfSatIds += weight * i;
    }

    emit(relTarget, weightedSumOfSatIds / sumWeight);

    // calculate required orientation
    inet::Quaternion newOrientation = inet::Quaternion::rotationFromTo(
            inet::Coord(1, 0, 0), weightedSumOfSatPosition);
    return newOrientation;
}

void MeanTracking::initialize() {
    INodeTracking::initialize();
    this->_trackingNodeIds.first = par("startNodeId");
    this->_trackingNodeIds.second = par("stopNodeId");
    this->_swipeStart = new omnetpp::cMessage("startSwipeTimer");
    this->_swipeStop = new omnetpp::cMessage("stopSwipeTimer");
    this->_exponent = par("exponent");

    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());

    auto cpManager = ContactPlanManager::getInstance();

    auto startContact = cpManager->getNextContactFor(groundstation->getNodeNo(),
            this->_trackingNodeIds.first);
    if (startContact != nullptr) {
        this->scheduleAt(startContact->startTime, this->_swipeStart);
    }

}

void MeanTracking::handleMessage(cMessage *message) {
    if (message == this->_swipeStart) {
        this->start();
    } else if (message == this->_swipeStop) {
        stop();
        auto cpManager = ContactPlanManager::getInstance();
        auto groundstation = check_and_cast<GroundStation*>(
                this->getParentModule());
        auto startContact = cpManager->getNextContactFor(
                groundstation->getNodeNo(), this->_trackingNodeIds.first);
        if (startContact != nullptr) {
            this->scheduleAt(startContact->startTime, this->_swipeStart);
        }
        auto stopContact = cpManager->getNextContactFor(
                groundstation->getNodeNo(), this->_trackingNodeIds.second);
        if (stopContact != nullptr) {
            this->scheduleAt(stopContact->endTime, this->_swipeStop);
        }

    }
}

void MeanTracking::finish() {
    cancelAndDelete(this->_swipeStart);
    cancelAndDelete(this->_swipeStop);

}

} //namespace
