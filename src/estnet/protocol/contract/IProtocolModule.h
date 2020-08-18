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

#ifndef __PROTOCOLS__I_PROTOCOL_MODULE_H__
#define __PROTOCOLS__I_PROTOCOL_MODULE_H__

#include <inet/common/packet/Packet.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * Interface for plugging different DTN protocols
 * into the same simulation setup.
 */
class ESTNET_API IProtocolModule {
    /**
     * checks whether the protocol has new packets available
     * returns true if new packet available, false otherwise
     */
    virtual bool dataFromProtocol() = 0;

    /**
     * receives new packet from protocol
     * @returns pointer to new packet
     */
    virtual omnetpp::cMessage* recvFromProtocol() = 0;

    /**
     * sends the given application packet to the protocol.
     * the expectation is that the protocol will pass the
     * packet to its sending interface and make a routing
     * decision. Afterwards @dataFromProtocol should return
     * true until @recvFromProtocol is called to grab a
     * BasicRadioFrame encapsulating the application packet.
     * BasicRadioFrame is expected to have all information
     * set, so that the routing decision can be inferred.
     * A BasicApplicationPacket could also be the response
     * if the application packet given as a argument here
     * is addressed to the the same node and no routing is
     * necessary, because it can be delivered back to us
     * directly.
     * @param appPacket the packet to give to the protocol
     */
    virtual void processPacketFromUpperLayer(inet::Packet *appPacket) = 0;

    /**
     * sends the given radio frame packet to the protocol.
     * the expectation is that the protocol will pass the
     * frame to its receiving interface and make a routing
     * decision. Afterwards @dataFromProtocol should return
     * true until @recvFromProtocol is called to grab a
     * BasicRadioFrame or BasicApplicationPacket.
     * BasicRadioFrame would mean the frame is going to be
     * forwarded to another node.
     * BasicApplicationPacket would mean the frame is
     * destined for this node and will be delived to the
     * application directly.
     * @param radioFrame the frame to give to the protocol
     */
    virtual void processPacketFromLowerLayer(inet::Packet *radioFrame) = 0;
};

}  // namespace estnet

#endif
