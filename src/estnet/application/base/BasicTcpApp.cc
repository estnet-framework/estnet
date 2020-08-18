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

#include "BasicTcpApp.h"

#include <inet/networklayer/contract/ipv4/Ipv4Address.h>
#include <inet/transportlayer/contract/tcp/TcpCommand_m.h>

#include "estnet/common/AddressUtils.h"
#include "estnet/common/StlUtils.h"

namespace estnet {

Define_Module(BasicTcpApp);

void BasicTcpApp::initialize(int stage) {
    BasicApp::initialize(stage);
    if (stage == 0) {
        this->_startMsg = new omnetpp::cMessage("startMsg");
        this->scheduleAt(0, this->_startMsg);
    }
}

BasicTcpApp::~BasicTcpApp() {
    this->cancelAndDelete(this->_startMsg);
}

void BasicTcpApp::bindSocket() {
    for (auto id : this->_destinationNodeNumbers) {
        this->_serverSocket.setOutputGate(gate(APP_OUT_GATE_NAME));
        inet::Ipv4Address localAddr = getIpAddressOfNode(id, 1);
        this->_serverSocket.bind(localAddr, this->_port);
        this->_serverSocket.listen();
        this->_serverSocketBind = true;
    }
}

void BasicTcpApp::connectSocket(unsigned int destNodeNo,
        inet::Ipv4Address dest) {
    inet::TcpSocket *socket = nullptr;
    if (!contains(this->_clientSockets, destNodeNo)) {
        socket = new inet::TcpSocket();
        this->_clientSockets.emplace(destNodeNo, socket);
    } else {
        socket = this->_clientSockets[destNodeNo];
        socket->renewSocket();
    }
    socket->setOutputGate(gate(APP_OUT_GATE_NAME));
    socket->connect(dest, this->_port);
}

void BasicTcpApp::handleMessage(omnetpp::cMessage *msg) {
    BasicApp::handleMessage(msg);

    if (this->_startMsg == msg) {
        if (!this->_serverSocketBind) {
            this->bindSocket();
        }
    } else if (msg->getKind() == inet::TCP_I_PEER_CLOSED) {
        // we close too
        msg->setName("close");
        msg->setKind(inet::TCP_C_CLOSE);
        send(msg, APP_OUT_GATE_NAME);
    } // else if (msg->getKind() == inet::TCP_I_DATA || msg->getKind() ==
      // inet::TCP_I_URGENT_DATA) {
      // cPacket *pk = PK(msg);
      // delete msg;
      //}
}

void BasicTcpApp::sendPacket(inet::Packet *packet) {
    if (!this->_serverSocketBind) {
        this->bindSocket();
    }

    unsigned int destNodeNo = packet->getTag<DestNodeIdTag>()->getDestNodeId();
    inet::Ipv4Address destAddr = getIpAddressOfNode(destNodeNo, 1);
    if (!contains(this->_clientSockets, destNodeNo)) {
        this->connectSocket(destNodeNo, destAddr);
    } else if (this->_clientSockets[destNodeNo]->getState()
            != inet::TcpSocket::CONNECTED) {
        this->connectSocket(destNodeNo, destAddr);
    }

    // actually send packet
    this->_clientSockets[destNodeNo]->send(packet);
}

}  // namespace estnet
