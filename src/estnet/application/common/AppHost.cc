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

#include "AppHost.h"

namespace estnet {

Define_Module(AppHost)

constexpr char AppHost::FROM_APP_GATE_NAME[];
constexpr char AppHost::TO_APP_GATE_NAME[];
constexpr char AppHost::FROM_PROTOCOL_GATE_NAME[];
constexpr char AppHost::TO_PROTOCOL_GATE_NAME[];

omnetpp::simsignal_t AppHost::sentPkSignal = registerSignal("sentPk");
omnetpp::simsignal_t AppHost::rcvdPkSignal = registerSignal("rcvdPk");

AppHost::AppHost() {
}

int AppHost::numInitStages() const {
    return 1;
}
void AppHost::initialize(int stage) {
    //nothing to do here
}

void AppHost::handleMessage(omnetpp::cMessage *message) {
    if (message->arrivedOn(FROM_APP_GATE_NAME)) {
        // we got a new packet to be send out from the upper layer
        this->receivedFromApp(omnetpp::check_and_cast<inet::Packet*>(message));
    } else if (message->arrivedOn(FROM_PROTOCOL_GATE_NAME)) {
        // we got a new frame to be handled from the lower layer
        this->receivedFromProtocolModule(
                omnetpp::check_and_cast<inet::Packet*>(message));
    } else {
        throw omnetpp::cRuntimeError("Unexpected message received");
    }
}

void AppHost::receivedFromProtocolModule(inet::Packet *pkt) {
    emit(rcvdPkSignal, pkt);

    auto appHostHeader = pkt->popAtFront<AppHostHeader>();
    unsigned int sourceNodeNo = appHostHeader.get()->getSourceNodeID();
    //add tag for appHostHeader information
    auto srcNodeIdTag = pkt->addTagIfAbsent<SrcNodeIdTag>();
    srcNodeIdTag->setSrcNodeId(sourceNodeNo);

    sendToApp(pkt);
}

void AppHost::sendToProtocolModule(inet::Packet *pkt) {
    emit(sentPkSignal, pkt);

    this->send(pkt, TO_PROTOCOL_GATE_NAME);
}

void AppHost::receivedFromApp(inet::Packet *pkt) {
    //add AppHostHeader to packet
    auto destTag = pkt->getTag<DestNodeIdTag>();
    auto destId = destTag->getDestNodeId();

    unsigned int sourceId = this->par("nodeNo");
    auto header = inet::makeShared<AppHostHeader>();
    header->setDestNodeID(destId);
    header->setSourceNodeID(sourceId);
    pkt->insertAtFront(header);

    //add AppHostHeader to packet
    auto numHopsHeader = inet::makeShared<NumHopsHeader>();
    numHopsHeader->setNumHops(0);
    pkt->insertAtBack(numHopsHeader);

    //send packet to protocol module
    sendToProtocolModule(pkt);
}

void AppHost::sendToApp(inet::Packet *pkt) {
    auto appHeader = pkt->peekAtFront<AppHeader>();
    uint16_t destAppId = appHeader.get()->getDestAppID();
    int numApps = this->par("numApps");
    // destAppId is zero based, numApps not
    if (destAppId + 1 > numApps) {
        throw omnetpp::cRuntimeError(
                "The destinationAppId does not exist. Are there the same number of apps on all nodes or are they correctly mapped if not?");
    } else {
        this->send(pkt, TO_APP_GATE_NAME, destAppId);
    }
}

}  // namespace estnet
