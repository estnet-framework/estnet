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

#ifndef __APPS__BASIC_UDP_APP_H__
#define __APPS__BASIC_UDP_APP_H__

#include <omnetpp.h>
#include <inet/transportlayer/contract/udp/UdpSocket.h>

#include "estnet/application/base/BasicApp.h"

namespace estnet {

/** An implementation of ~BasicApp with
 *  UDP protocol addressing support.
 */
class ESTNET_API BasicUdpApp: public BasicApp {
protected:
    virtual ~BasicUdpApp();
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief message receiver function */
    virtual void handleMessage(omnetpp::cMessage*) override;
    /** @brief binds an UDP socket */
    virtual void bindSocket();
    /** @brief called with a generated packet to be sent out */
    virtual void sendPacket(inet::Packet *packet) override;

private:
    omnetpp::cMessage *_startMsg;
    inet::UdpSocket _socket;
    bool _socketBind = false;
    int _port = 42;
};

}  // namespace estnet

#endif
