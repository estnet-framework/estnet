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

#include "DirectProtocolModuleBase.h"

namespace estnet {

Define_Module(DirectProtocolModuleBase);

void DirectProtocolModuleBase::initialize(int stage) {
    ProtocolModuleBase::initialize(stage);
    if (stage == 0) {
        this->useBroadcasts = this->par("useBroadcasts").boolValue();
    }
}

void DirectProtocolModuleBase::processPacketFromUpperLayer(
        inet::Packet *appPacket) {
    unsigned int nextHopNodeNo;
    auto appHostHeader = appPacket->peekAtFront<AppHostHeader>();
    long destNodeId = appHostHeader->getDestNodeID();

    if (this->useBroadcasts) {
        nextHopNodeNo = 0;
    } else {
        nextHopNodeNo = destNodeId;
    }

    auto nextHopTag = appPacket->addTagIfAbsent<NextHopReq>();
    nextHopTag->setNextHopNodeNo(nextHopNodeNo);

    response = appPacket;
}

void DirectProtocolModuleBase::processPacketFromLowerLayer(
        inet::Packet *radioFrame) {
    unsigned int nextHopNodeNo;
    auto appHostHeader = radioFrame->peekAtFront<AppHostHeader>();
    unsigned int destNodeId = appHostHeader->getDestNodeID();

    if (destNodeId == this->_nodeNo || destNodeId == 0) {
        response = radioFrame;
    } else {
        if (this->useBroadcasts) {
            nextHopNodeNo = 0;
        } else {
            nextHopNodeNo = destNodeId;
        }

        auto nextHopTag = radioFrame->addTagIfAbsent<NextHopReq>();
        nextHopTag->setNextHopNodeNo(nextHopNodeNo);

        response = radioFrame;
    }
}

bool DirectProtocolModuleBase::dataFromProtocol() {
    return response != nullptr;
}

omnetpp::cPacket* DirectProtocolModuleBase::recvFromProtocol() {
    omnetpp::cPacket *rtn = response;
    response = nullptr;
    return rtn;
}

}  // namespace estnet
