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

#include "ContactPlanBasedNodeTracking.h"

//#include <inet/common/ModuleAccess.h>

#include "estnet/common/ModuleAccess.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/common/node/NodeRegistry.h"

namespace estnet {

Define_Module(ContactPlanBasedNodeTracking);

void ContactPlanBasedNodeTracking::initialize() {
    INodeTracking::initialize();
    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    groundstation->enableContactPlanTracking();
    this->subscribeToSatellitePositions();

}

void ContactPlanBasedNodeTracking::finish() {
    this->unsubscribeFromSatellitePositions();
}

void ContactPlanBasedNodeTracking::start() {
    INodeTracking::start();
    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    // get node to track
    std::set<unsigned int> nodes = groundstation->nodesAvailableToBeTracked();

    // if we're currently not tracking anyone, start now
    if (this->_currentlyTrackingNodeNo == 0) {
        this->_currentlyTrackingNodeNo = *nodes.begin();
        StaticTerrestrialMobility *m = groundstation->getMobility();
        m->setCurrentAngularPosition(this->track());

        EV_INFO << "GroundStation " << groundstation->getNodeNo()
                       << ": Starting to track satellite "
                       << _currentlyTrackingNodeNo << omnetpp::endl;
    }

}

void ContactPlanBasedNodeTracking::stop() {
    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    // get node to track
    std::set<unsigned int> nodes = groundstation->nodesAvailableToBeTracked();

    // if we're loosing connection to the node we're tracking, stop tracking them
    if (nodes.find(_currentlyTrackingNodeNo) == nodes.end()) {
        this->_currentlyTrackingNodeNo = 0;

        EV_INFO << "GroundStation " << groundstation->getNodeNo()
                       << ": Stopping to track satellite "
                       << _currentlyTrackingNodeNo << omnetpp::endl;
        // check if we can switch to track another node
        if (!nodes.empty()) {
            unsigned int new_tracked_node_no = *nodes.begin();
            this->_currentlyTrackingNodeNo = new_tracked_node_no;
            StaticTerrestrialMobility *m = groundstation->getMobility();
            m->setCurrentAngularPosition(this->track());
            EV_INFO << "GroundStation " << groundstation->getNodeNo()
                           << ": Starting to track satellite "
                           << new_tracked_node_no << omnetpp::endl;
        } else {
            turnToDefault();
            INodeTracking::stop();
        }
    }
}

inet::Quaternion ContactPlanBasedNodeTracking::getNewOrientation() {

    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    // get node to track
    std::set<unsigned int> nodes = groundstation->nodesAvailableToBeTracked();

    if (nodes.empty()) {
        INodeTracking::stop();
        return turnToDefault();
    }
    // if we're currently not tracking anyone, start now
    if (this->_currentlyTrackingNodeNo == 0) {
        this->_currentlyTrackingNodeNo = *nodes.begin();
        EV_INFO << "GroundStation " << groundstation->getNodeNo()
                       << ": Starting to track satellite "
                       << _currentlyTrackingNodeNo << omnetpp::endl;
    }
    return this->track();
}

inet::Quaternion ContactPlanBasedNodeTracking::track() {
    Satellite *satelliteNode = NodeRegistry::getInstance()->getSatellite(
            _currentlyTrackingNodeNo);
    if (satelliteNode == nullptr) {
        throw omnetpp::cRuntimeError(
                "Tracking something that is not a satellite");
    }
    SatMobility *mobility =
            dynamic_cast<SatMobility*>(satelliteNode->getMobility());
    if (mobility == nullptr) {
        throw omnetpp::cRuntimeError(
                "Tracking something that is not SatMobility");
    }
    inet::Coord targetPositionECI = mobility->getCurrentPositionWithoutSignal();

    auto groundstation = check_and_cast<GroundStation*>(
            this->getParentModule());
    StaticTerrestrialMobility *m = groundstation->getMobility();
    inet::Coord ourPositionECI = m->getCurrentPosition();

    inet::Quaternion newOrientation = inet::Quaternion::rotationFromTo(
            inet::Coord(1, 0, 0), targetPositionECI - ourPositionECI);
    return newOrientation;
}

void ContactPlanBasedNodeTracking::subscribeToSatellitePositions() {
    auto subscriptionModule = getModuleFromPar<cModule>(
            this->par("subscriptionModule"), this);
    subscriptionModule->subscribe(inet::IMobility::mobilityStateChangedSignal,
            this);
}

void ContactPlanBasedNodeTracking::unsubscribeFromSatellitePositions() {
    // NOTE: lookup the module again because it may have been deleted first
    auto subscriptionModule = getModuleFromPar<cModule>(
            this->par("subscriptionModule"), this, false);
    if (subscriptionModule != nullptr) {
        subscriptionModule->unsubscribe(
                inet::IMobility::mobilityStateChangedSignal, this);
    }
}

void ContactPlanBasedNodeTracking::receiveSignal(cComponent *source,
        omnetpp::simsignal_t signal, cObject *object, cObject *details) {
    Enter_Method_Silent
    ();
    if (this->_currentlyTrackingNodeNo == 0) {
        // we 're not tracking, so get out of here quickly
        return;
    }
    if (signal == inet::IMobility::mobilityStateChangedSignal) {
        SatMobility *mobility = dynamic_cast<SatMobility*>(object);
        if (mobility == nullptr) {
            throw omnetpp::cRuntimeError(
                    "Got signal from something that is not SatMobility");
        }
        unsigned int nodeNo = findContainingNode(mobility)->par("nodeNo");
        if (nodeNo == this->_currentlyTrackingNodeNo) {
            // got update from the node we're tracking
            auto groundstation = check_and_cast<GroundStation*>(
                    this->getParentModule());
            StaticTerrestrialMobility *m = groundstation->getMobility();
            m->setCurrentAngularPosition(this->track());
        }
    }
}

} //namespace
