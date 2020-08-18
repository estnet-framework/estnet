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

#include "BasicUdpApp.h"

#include <inet/networklayer/contract/ipv4/Ipv4Address.h>

#include "estnet/common/AddressUtils.h"

namespace estnet {

Define_Module(BasicUdpApp);

void BasicUdpApp::initialize(int stage) {
    BasicApp::initialize(stage);
    if (stage == 0) {
        this->_startMsg = new omnetpp::cMessage("startMsg");
        this->scheduleAt(0, this->_startMsg);
    }
}

BasicUdpApp::~BasicUdpApp() {
    this->cancelAndDelete(this->_startMsg);
}

void BasicUdpApp::bindSocket() {
    for (auto id : this->_destinationNodeNumbers) {
        this->_socket.setOutputGate(gate(APP_OUT_GATE_NAME));
        inet::Ipv4Address localAddr = getIpAddressOfNode(id, 1);
        this->_socket.bind(localAddr, this->_port);
        this->_socketBind = true;
    }
}

void BasicUdpApp::handleMessage(omnetpp::cMessage *msg) {
    if (this->_startMsg == msg) {
        if (!this->_socketBind) {
            this->bindSocket();
        }
    } else {
        BasicApp::handleMessage(msg);
    }
}

void BasicUdpApp::sendPacket(inet::Packet *packet) {
    if (!this->_socketBind) {
        this->bindSocket();
    }

    unsigned int destNodeNo = packet->getTag<DestNodeIdTag>()->getDestNodeId();
    inet::Ipv4Address destAddr = getIpAddressOfNode(destNodeNo, 1);

    // actually send packet
    this->_socket.sendTo(packet, destAddr, this->_port);
}

}  // namespace estnet
