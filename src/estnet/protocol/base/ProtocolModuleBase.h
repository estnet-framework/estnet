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

#ifndef __PROTOCOLS__PROTOCOL_MODULE_BASE_H__
#define __PROTOCOLS__PROTOCOL_MODULE_BASE_H__

#include <omnetpp.h>
#include <tuple>
#include <set>
#include <vector>

#include "estnet/protocol/contract/IProtocolModule.h"

namespace estnet {

const short PROTOCOL_MODULE_BUFFER_TIMEOUT = 3;
const short PROTOCOL_MODULE_PACKET_THROWAWAY = 4;

/**
 * Augments @IProtocolModule with methods
 * to handle the interaction with connected
 * apps and radios. The original interface of
 * @IProtocolModule remains unimplemented and
 * is expected to be handled by the subclasses
 */
class ESTNET_API ProtocolModuleBase: public IProtocolModule,
        public omnetpp::cSimpleModule {
protected:
    virtual ~ProtocolModuleBase();
    /** @brief returns number of initalization stages */
    virtual int numInitStages() const override;
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief receives packets from radios or apps */
    virtual void handleMessage(omnetpp::cMessage *message) override;

    static constexpr char FROM_APP_GATE_NAME[] = "fromAppHost";
    static constexpr char TO_APP_GATE_NAME[] = "toAppHost";
    static constexpr char FROM_RADIO_GATE_NAME[] = "ifIn";
    static constexpr char TO_RADIO_GATE_NAME[] = "ifOut";

    static omnetpp::simsignal_t rcvdPacketFromHL;
    static omnetpp::simsignal_t sentPacketToHL;
    static omnetpp::simsignal_t incomingFrame;
    static omnetpp::simsignal_t incomingDiscardedFrame;
    static omnetpp::simsignal_t forwardedFrame;
    static omnetpp::simsignal_t outgoingFrame;
    static omnetpp::simsignal_t outgoingDiscardedFrame;
    static omnetpp::simsignal_t droppedPk;
    static omnetpp::simsignal_t bufferedPk;
    static omnetpp::simsignal_t bufferedFrame;
    static omnetpp::simsignal_t bufferingTime;
    static omnetpp::simsignal_t bufferLength;

    /**
     * Interface functions to be provided
     */

    /**
     * checks whether the protocol has new packets available
     * returns true if new packet available, false otherwise
     */
    virtual bool dataFromProtocol() override = 0;
    /**
     * receives new packet from protocol
     * @returns pointer to new packet
     */
    virtual omnetpp::cMessage* recvFromProtocol() override = 0;
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
    virtual void processPacketFromUpperLayer(inet::Packet *appPacket)
            override = 0;
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
    virtual void processPacketFromLowerLayer(inet::Packet *radioFrame)
            override = 0;
    /**
     * @brief called to synchronize the simulation time with the protocol
     */
    virtual void sendTimeToProtocol() {
    }
    ;
    /**
     * @brief called to give the routing table to the protocol at initialization
     */
    //virtual void sendRoutingTableToProtocol(const RoutingTable& routingTable) {};

    /**
     * Base implementation provided to all implementations
     */

    /** @brief called when the protocol returns a radio frame */
    virtual void receivedRadioFrameFromProtocol(inet::Packet *radioFrame);
    /** @brief called when the protocol returns an app packet */
    virtual void receivedFromProtocol(inet::Packet *appPacket);
    /** @brief called when the app send us a packet */
    virtual void receivedFromApp(inet::Packet *message);
    /** @brief called when the radio send us a frame */
    virtual void receivedFromRadio(inet::Packet *message);
    /** @brief called to deliver a packet to an app */
    virtual void sendToApp(omnetpp::cMessage *message);
    /** @brief called to deliver a frame to a radio */
    virtual void sendToRadio(inet::Packet *frame);

    virtual void handleBufferTimeout(omnetpp::cMessage *message);

    /** @brief waits until the protocol responds to a frame or packet
     *  and then calls the appriopriate received functions
     */
    virtual void waitForProtocolResponse(omnetpp::cMessage *message);

    /** @brief whether the protocol module should delete radio frames for us */
    virtual bool deleteReceivedRadioFrame() const {
        return true;
    }

    /** @brief returns the current sequence number */
    long getCurrentSequenceNumber() {
        return this->_nextPacketSequenceNumber;
    }
    /** @brief returns the current sequence number and increment */
    long getNextSequenceNumber() {
        return this->_nextPacketSequenceNumber++;
    }

    unsigned int _nodeNo;
    std::set<std::tuple<unsigned int, long>> _seenPackets;
    bool _dropAlreadySeenFrames;
private:
    long _nextPacketSequenceNumber = 1;
    std::vector<omnetpp::cMessage*> _bufferedTimeouts;
};

}  // namespace estnet

#endif
