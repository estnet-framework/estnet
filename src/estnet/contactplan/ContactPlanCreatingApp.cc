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

#include "ContactPlanCreatingApp.h"
#include "estnet/common/AddressUtils.h"

#include "inet/linklayer/common/MacAddressTag_m.h"
#include <inet/common/packet/Packet.h>

namespace estnet {

Define_Module(ContactPlanCreatingApp);

int ContactPlanCreatingApp::numInitStages() const {
    return 1;
}

void ContactPlanCreatingApp::initialize(int stage) {
    if (stage == 0) {
        this->_nodeId = this->par("nodeNo");
        this->_numReceived = 0;
        this->_startTime = par("startTime").doubleValue();
        this->_stopTime = par("stopTime").doubleValue();
        // schedule packet sending
        this->_scheduleMsg = nullptr;
        if (par("sending").boolValue()) {
            this->_scheduleMsg = new omnetpp::cMessage("appSchedule");
            this->scheduleAt(this->_startTime, this->_scheduleMsg);
        }
    } else {
        throw omnetpp::cRuntimeError("Unknown initialization stage");
    }
}

void ContactPlanCreatingApp::finish() {
    EV_INFO << "Node " << this->_nodeId << ": Received " << this->_numReceived
                   << " packets" << omnetpp::endl;
    omnetpp::cSimpleModule::finish();
}

ContactPlanCreatingApp::~ContactPlanCreatingApp() {
    this->cancelAndDelete(this->_scheduleMsg);
}

void ContactPlanCreatingApp::handleMessage(omnetpp::cMessage *msg) {
    if (msg == this->_scheduleMsg) {
        unsigned int destNodeId = par("destinationNodeNo").intValue();
        unsigned int payloadSize = par("payloadSize").intValue();

        auto pkt = new inet::Packet("ContactPlanCreatingApp message");
        pkt->setByteLength(payloadSize);
        auto macAddressBase = pkt->addTag<inet::MacAddressTagBase>();
        macAddressBase->setSrcAddress(getMacAddressOfNode(this->_nodeId, 1));
        macAddressBase->setDestAddress(getMacAddressOfNode(destNodeId, 1));
        this->send(pkt, "lowerLayerOut");

        double sendInterval = par("sendInterval").doubleValue();
        omnetpp::simtime_t d = omnetpp::simTime() + sendInterval;
        if (sendInterval > 0
                && (this->_stopTime < SIMTIME_ZERO || d < this->_stopTime)) {
            this->scheduleAt(d, this->_scheduleMsg);
        }
    } else if (msg->getArrivalGate() == gate("lowerLayerIn")) {
        EV_INFO << "Node " << this->_nodeId << ": Received packet"
                       << omnetpp::endl;
        this->_numReceived++;
        delete msg;
    } else {
        throw omnetpp::cRuntimeError("Unknown message");
    }
}

}  // namespace estnet
