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

#ifndef __PROTOCOLS__EXTERNAL_PROTOCOL_MODULE_BASE_H__
#define __PROTOCOLS__EXTERNAL_PROTOCOL_MODULE_BASE_H__

#include <tuple>
#include <map>
#include <rapidjson/document.h>

#include <inet/common/packet/Packet.h>

#include "ProtocolModuleBase.h"
#include "estnet/contactplan/common/Contacts.h"

namespace estnet {

/**
 * Implementation of ~ProtocolModuleBase, which implements
 * a JSON protocol to communicate with external protocol
 * implementations.
 */
class ESTNET_API ExternalProtocolModuleBase: public ProtocolModuleBase {
protected:
    /**
     * Interface functions to be provided
     */

    /** @brief checks whether the external process has a response */
    virtual bool dataFromProtocol() override = 0;
    /** @brief sends a command to the external protocol process */
    virtual void sendToProtocol(const std::string &commandString) = 0;
    /** @brief receives a command from the external protocol process */
    virtual void recvFromProtocol(std::string &commandString) = 0;

    /**
     * Base implementation provided to all implementations
     */

    /**
     * receives new packet from the external protocol
     * @returns pointer to new packet
     */
    virtual omnetpp::cMessage* recvFromProtocol() override;
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
    virtual void processPacketFromUpperLayer(inet::Packet *appPacket) override;
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
     * destined for this node and will be delivered to the
     * application directly.
     * @param radioFrame the frame to give to the protocol
     */
    virtual void processPacketFromLowerLayer(inet::Packet *radioFrame) override;

    /** @brief called to synchronize the simulation time with the protocol */
    virtual void sendTimeToProtocol() override;
    /** @brief called to give the routing table to the protocol at initialization */
    //virtual void sendRoutingTableToProtocol(const RoutingTable& routingTable)
    //        override;
    /** @brief builds a time synchronization command */
    virtual void buildTimeCommand(const std::string &command,
            std::string &commandString);
    /** @brief builds a 'send packet' command */
    virtual void buildSendCommand(const std::string &command,
            uint64_t sequenceNumber, unsigned int sourceAppId,
            unsigned int destAppId, unsigned int sourceNodeNo,
            unsigned int destinationNodeNo, const uint8_t *payload,
            size_t payloadSize, std::string &commandString);
    /** @brief builds a 'received packet' command */
    virtual void buildReceiveCommand(const std::string &command,
            uint64_t sequenceNumber, unsigned int sourceAppId,
            unsigned int destAppId, unsigned int sourceNodeNo,
            unsigned int destinationNodeNo, const uint8_t *payload,
            size_t payloadSize, std::string &commandString);
    /** @brief builds a buffer timeout response for ~ProtocolModuleBase */
    virtual omnetpp::cMessage* getTimeoutMessage(rapidjson::Document &d);
    /** @brief builds the respective radio frame for a 'send packet' response from the protocol */
    virtual inet::Packet* getRadioFrame(rapidjson::Document &d);
    /** @brief grabs the respective app packet for a 'receive packet' response from the protocol */
    virtual inet::Packet* getAppPacket(rapidjson::Document &d);
    /** @brief whether the protocol module should delete radio frames for us */
    virtual bool deleteReceivedRadioFrame() const override {
        return false;
    }
private:
    std::map<SequenceNumber, inet::Packet*> _appPackets;
    std::map<SequenceNumber, inet::Packet*> _radioFrames;
};

}  // namespace estnet

#endif
