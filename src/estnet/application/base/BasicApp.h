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

#ifndef __ESTNET_BASICAPP_H_
#define __ESTNET_BASICAPP_H_

#include <inet/common/ProtocolTag_m.h>
#include <inet/linklayer/common/MacAddressTag_m.h>
#include <inet/common/packet/chunk/ByteCountChunk.h>
#include <inet/common/packet/Packet.h>

#include "estnet/application/contract/IApp.h"
#include "estnet/application/common/AppHeader_m.h"
#include "estnet/application/common/AppHostHeader_m.h"
#include "estnet/application/common/DestNodeIdTag_m.h"
#include "estnet/application/common/SrcNodeIdTag_m.h"
#include "estnet/protocol/common/NumHopsHeader_m.h"
#include "estnet/common/AddressUtils.h"
#include "estnet/common/node/NodeRegistry.h"

namespace estnet {

const short BASIC_APP_PACKET_TYPE = 1;
const char APP_IN_GATE_NAME[] = "appIn";
const char APP_OUT_GATE_NAME[] = "appOut";

struct PacketInformation {
    unsigned int nodeId;
    unsigned int appId;
    unsigned long pktId;
    int numHops;
    bool operator <(const PacketInformation &b) const {
        if (b.nodeId != nodeId) {
            return (nodeId < b.nodeId);
        } else if (b.appId != appId) {
            return (appId < b.appId);
        }
        return (pktId < b.pktId);
    }
};

/**
 *  Basic app class with multiple nodes as destination
 */
class ESTNET_API BasicApp: public omnetpp::cSimpleModule, public IApp {
public:
    virtual ~BasicApp();

protected:
    std::vector<int> _destinationNodeNumbers;
    omnetpp::simtime_t _startTime;
    omnetpp::simtime_t _stopTime;
    omnetpp::cMessage *_scheduleMsg;
    bool _printNoPackets;
    bool _printReceivedPacketsNo;
    bool _printSentPacketsNo;
    bool _printMissingPacketsNo;
    std::vector<PacketInformation> _packetsGenerated;
    std::vector<PacketInformation> _packetsReceived;

    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief returns number of initalization stages */
    virtual int numInitStages() const override;
    /** @brief message receiver function */
    virtual void handleMessage(omnetpp::cMessage*) override;
    /** @brief cleanup function */
    virtual void finish() override;

    /** @brief called to set destination node of generated packet */
    virtual void generatePacketDestinationNodeNumbers();
    /** @brief called to set generated packet kind */
    virtual short generatePacketKind();
    /** @brief called to set payload of generated packet */
    virtual inet::Packet* generatePacketPayload();
    /** @brief called to figure out when next packet needs to be generated */
    virtual bool nextPacketTime(omnetpp::simtime_t &t);
    /** @brief called to schedule the next packet. */
    virtual void scheduleNextPacket(omnetpp::simtime_t nextTime = -1);
    /** @brief called with a generated packet to be sent out */
    virtual void sendPacket(inet::Packet *packet);
    /** @brief called with a received packets */
    virtual inet::Packet* recvPacket(omnetpp::cMessage *msg);

    static omnetpp::simsignal_t sentPkSignal;
    static omnetpp::simsignal_t rcvdPkSignal;

    /** @brief internal method to send packets */
    virtual void sendPacketInternal();
    /** @brief internal method to receive packets */
    virtual void recvPacketInternal(omnetpp::cMessage *msg);

};

}  // namespace estnet

#endif
