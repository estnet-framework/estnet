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

#include "NodeBase.h"

namespace estnet {

class ESTNET_API ContactPlanManager {
public:
    static ContactPlanManager* getInstance();
    int getDownlinkCoverageForNode(unsigned int nodeId);
    int getUplinkCoverageForNode(unsigned int nodeId);
};

int NodeBase::numInitStages() const {
    return 1;
}

void NodeBase::initialize(int stage) {
    if (stage == 0) {
        this->uplinkCoverageId = registerSignal("uplinkCoverage");
        this->downlinkCoverageId = registerSignal("downLinkCoverage");
        this->_nodeNo = this->par("nodeNo").intValue();
    }
}

std::vector<IApp*> NodeBase::getApps() const {
    std::vector<IApp*> apps;
    auto networkHost = this->getSubmodule("networkHost");
    int numApps = networkHost->par("numApps").intValue();
    for (int i = 0; i < numApps; i++) {
        IApp *app = omnetpp::check_and_cast<IApp*>(
                networkHost->getSubmodule("appWrapper", i)->getSubmodule(
                        "app"));
        apps.push_back(app);
    }
    return apps;
}

bool NodeBase::contactManagementEnabled() const {
    std::string test(this->par("nodeContactManager").stringValue());
    return !test.empty();
}

NodeContactManager* NodeBase::getNodeContactManager() const {
    return dynamic_cast<NodeContactManager*>(this->getSubmodule("networkHost")->getSubmodule(
            this->par("nodeContactManager").stringValue()));
}

NodeContactManager* NodeBase::getNodeContactManagerInternal() const {
    // just sanity checking, so we don't have to do nullptr checks everywhere
    NodeContactManager *nodeContactManager = this->getNodeContactManager();
    if (nodeContactManager == nullptr) {
        throw omnetpp::cRuntimeError("getNodeContactManager returned nullptr");
    }
    return nodeContactManager;
}

inet::IMobility* NodeBase::getMobility() const {
    return dynamic_cast<inet::IMobility*>(this->getSubmodule("networkHost")->getSubmodule(
            "mobility"));
}

void NodeBase::addContactTo(unsigned int other_node_no) {
    if (this->contactManagementEnabled()) {
        this->getNodeContactManagerInternal()->addContactTo(other_node_no);
    }
}

void NodeBase::removeContactTo(unsigned int other_node_no) {
    if (this->contactManagementEnabled()) {
        this->getNodeContactManagerInternal()->removeContactTo(other_node_no);
    }
}

void NodeBase::addContactBetween(time_t startTime, time_t endTime,
        unsigned int node_no, unsigned int other_node_no, unsigned int bitrate,
        unsigned int range) {
    if (this->contactManagementEnabled()) {
        this->getNodeContactManagerInternal()->addContactBetween(startTime,
                endTime, node_no, other_node_no, bitrate, range);
    }
}

void NodeBase::bundleRouteNotification(unsigned int sourceNodeId,
        unsigned int destNodeId, long sequenceNumber,
        unsigned int nextHopNodeId, bool abandoned) {
    // will be delivered to application if destNodeId is us, nothing to do will be
    // delivered immediately without significant delays
    // otherwise update the sending timeout
    if (this->_nodeNo != destNodeId) {
        if (this->contactManagementEnabled()) {
            this->getNodeContactManagerInternal()->updateSendingTimeout(
                    sourceNodeId, sequenceNumber, nextHopNodeId, abandoned);
        }
    }
}

void NodeBase::waitForLimboDecisions(bool reset) {
    if (this->contactManagementEnabled()) {
        this->getNodeContactManagerInternal()->waitForLimboDecisions(reset);
    }
}

void NodeBase::finish() {
    omnetpp::cModule::finish();
}

}  // namespace estnet
