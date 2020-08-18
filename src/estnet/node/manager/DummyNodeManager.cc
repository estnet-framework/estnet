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

#include "DummyNodeManager.h"

namespace estnet {

Define_Module(DummyNodeManager);

void DummyNodeManager::initialize(int stage) {
    if (stage == 0) {
        this->_nodeNo = this->par("nodeNo");
    }
}

void DummyNodeManager::addContactTo(unsigned int other_node_no) {
    EV_INFO << "Node " << this->_nodeNo << " has new contact to "
                   << other_node_no << omnetpp::endl;
}

void DummyNodeManager::removeContactTo(unsigned int other_node_no) {
    EV_INFO << "Node " << this->_nodeNo << " lost contact to " << other_node_no
                   << omnetpp::endl;
}

void DummyNodeManager::addContactBetween(time_t startTime, time_t endTime,
        unsigned int node_no, unsigned int other_node_no, unsigned int bitrate,
        unsigned int range) {
    /// not printing potential contacts between all nodes
}

void DummyNodeManager::updateSendingTimeout(unsigned int sourceNodeId,
        long sequenceNumber, unsigned int nextHopNodeId, bool abandoned) {
    // not doing anything
}

void DummyNodeManager::waitForLimboDecisions(bool reset) {
    // not doing anything
}

}  // namespace estnet
