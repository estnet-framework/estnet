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

#include "ISwipeTracking.h"

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/contactplan/common/ContactPlanManager.h"

namespace estnet {

omnetpp::simsignal_t ISwipeTracking::relTarget = registerSignal("relTarget");

void ISwipeTracking::initialize() {
    INodeTracking::initialize();
    this->_trackingNodeIds.first = par("startNodeId");
    this->_trackingNodeIds.second = par("stopNodeId");
    this->_swipeStart = new omnetpp::cMessage("startSwipeTimer");
    this->_swipeStop = new omnetpp::cMessage("stopSwipeTimer");

    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());

    auto cpManager = ContactPlanManager::getInstance();

    auto startContact = cpManager->getNextContactFor(groundstation->getNodeNo(),
            this->_trackingNodeIds.first);
    if (startContact != nullptr) {
        this->scheduleAt(startContact->startTime, this->_swipeStart);
    }

}

void ISwipeTracking::handleMessage(cMessage *message) {
    if (message == this->_swipeStart) {
        this->start();
        updateTimeParameters();
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

    }
}

void ISwipeTracking::finish() {
    cancelAndDelete(this->_swipeStart);
    cancelAndDelete(this->_swipeStop);

}

void ISwipeTracking::updateTimeParameters() {
    auto cpManager = ContactPlanManager::getInstance();
    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    auto startContact = cpManager->getNextContactFor(groundstation->getNodeNo(),
            this->_trackingNodeIds.first);
    auto stopContact = cpManager->getNextContactFor(groundstation->getNodeNo(),
            this->_trackingNodeIds.second);
    if (stopContact != nullptr) {
        _T.first = GlobalJulianDate::getInstance().simTime2JulianDate(
                startContact->startTime);
        _T.second = GlobalJulianDate::getInstance().simTime2JulianDate(
                stopContact->endTime);
        // calculate stats of overfly
        _trackingTime = stopContact->endTime - startContact->startTime;
        double overflyingTime = 0;
        overflyingTime += startContact->endTime - startContact->startTime;
        overflyingTime += stopContact->endTime - stopContact->startTime;
        _deltaT = _trackingTime - overflyingTime / 2;

        this->scheduleAt(stopContact->endTime, this->_swipeStop);
    } else {
        // todo: check for simtime end?
    }
}

} //namespace
