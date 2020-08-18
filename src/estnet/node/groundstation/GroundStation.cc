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

#include "GroundStation.h"

#include <stdio.h>

#include "estnet/common/node/NodeRegistry.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/common/matrix/Matrix.h"
#include "estnet/global_config.h"
#include "estnet/node/tracking/contract/INodeTracking.h"

namespace estnet {

Define_Module(GroundStation);

int GroundStation::numInitStages() const {
    return 11;
}

void GroundStation::initialize(int stage) {
    NodeBase::initialize(stage);
    if (stage == 1) {
        // pointing up with the local x-axis by default
        NodeRegistry::getInstance()->addGroundStation(this);
    }
}

std::set<unsigned int> GroundStation::nodesAvailableToBeTracked() {
    return this->_nodesAvailableToBeTracked;
}

void GroundStation::enableContactPlanTracking() {
    _useContactPlanTracking = true;
}

StaticTerrestrialMobility* GroundStation::getMobility() const {
    return dynamic_cast<StaticTerrestrialMobility*>(this->getSubmodule(
            "networkHost")->getSubmodule("mobility"));
}

void GroundStation::addContactTo(unsigned int other_node_no) {
    NodeBase::addContactTo(other_node_no);
    if (!_useContactPlanTracking) {
        return;
    }

    // check if the contact is a satellite
    Satellite *satelliteNode = NodeRegistry::getInstance()->getSatellite(
            other_node_no);
    if (satelliteNode != nullptr) {
        // remembering who we can track
        this->_nodesAvailableToBeTracked.insert(other_node_no);
    }
    if (this->_nodesAvailableToBeTracked.size() > 0) {
        INodeTracking *tracking =
                dynamic_cast<INodeTracking*>(this->getSubmodule("nodeTracking"));
        tracking->start();
    }

}

void GroundStation::removeContactTo(unsigned int other_node_no) {
    NodeBase::removeContactTo(other_node_no);
    if (!_useContactPlanTracking) {
        return;
    }
    // remembering who we can track
    this->_nodesAvailableToBeTracked.erase(other_node_no);

    INodeTracking *tracking = dynamic_cast<INodeTracking*>(this->getSubmodule(
            "nodeTracking"));
    tracking->stop();
}

bool GroundStation::canCommunicateWithoutRadioWith(
        unsigned int other_node_no) const {
    // is this ground station connected to the other ground stations via Internet?
    bool isInternetConnected = this->par("internetConnection").boolValue();
    if (!isInternetConnected) {
        return false;
    }
    // check if we other ground station exists, other satellites will never be
    // able to do this
    GroundStation *otherGroundStation =
            NodeRegistry::getInstance()->getGroundStation(other_node_no);
    if (otherGroundStation == nullptr) {
        return false;
    }
    // check if the other ground station is also internet connected
    bool otherIsInternetConnected = otherGroundStation->par(
            "internetConnection").boolValue();
    return otherIsInternetConnected;
}

}  // namespace estnet
