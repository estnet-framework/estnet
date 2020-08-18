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

#ifndef __APPS__BASIC_TCP_APP_H__
#define __APPS__BASIC_TCP_APP_H__

#include <map>

#include <omnetpp.h>
#include <inet/transportlayer/contract/tcp/TcpSocket.h>
#include <inet/common/socket/SocketMap.h>
#include <inet/networklayer/contract/ipv4/Ipv4Address.h>

#include "estnet/application/base/BasicApp.h"

namespace estnet {

/**
 * An implementation of ~BasicApp with
 * TCP protocol addressing support
 */
class ESTNET_API BasicTcpApp: public BasicApp {
protected:
    virtual ~BasicTcpApp();
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief message receiver function */
    virtual void handleMessage(omnetpp::cMessage*) override;
    /** @brief binds a TCP socket */
    virtual void bindSocket();
    /** @brief connects a TCP socket */
    virtual void connectSocket(unsigned int destNodeNo, inet::Ipv4Address dest);

    /** @brief called with a generated packet to be sent out */
    virtual void sendPacket(inet::Packet *packet) override;

private:
    omnetpp::cMessage *_startMsg;
    inet::TcpSocket _serverSocket;
    inet::SocketMap _socketMap;
    std::map<unsigned int, inet::TcpSocket*> _clientSockets;
    bool _serverSocketBind = false;
    int _port = 42;
};

}  // namespace estnet

#endif
