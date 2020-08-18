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

#include "ProtocolModuleBase.h"

#include <inet/linklayer/common/MacAddressTag_m.h>
#include <inet/common/Ptr.h>
#include <inet/common/packet/chunk/ByteCountChunk.h>
#include <inet/common/ProtocolTag_m.h>
#include <inet/common/packet/tag/TagSet.h>
#include <inet/linklayer/acking/AckingMac.h>

#include "estnet/common/AddressUtils.h"
#include "estnet/common/StlUtils.h"
#include "estnet/common/node/NodeRegistry.h"
#include "estnet/contactplan/common/ContactPlanManager.h"
#include "estnet/protocol/common/NumHopsHeader_m.h"
#include "estnet/protocol/common/NextHopTag_m.h"

namespace estnet {

omnetpp::simsignal_t ProtocolModuleBase::rcvdPacketFromHL = registerSignal(
        "rcvdPacketFromHL");
omnetpp::simsignal_t ProtocolModuleBase::sentPacketToHL = registerSignal(
        "sentPacketToHL");
omnetpp::simsignal_t ProtocolModuleBase::incomingFrame = registerSignal(
        "incomingFrame");
omnetpp::simsignal_t ProtocolModuleBase::incomingDiscardedFrame =
        registerSignal("incomingDiscardedFrame");
omnetpp::simsignal_t ProtocolModuleBase::forwardedFrame = registerSignal(
        "forwardedFrame");
omnetpp::simsignal_t ProtocolModuleBase::outgoingFrame = registerSignal(
        "outgoingFrame");
omnetpp::simsignal_t ProtocolModuleBase::outgoingDiscardedFrame =
        registerSignal("outgoingDiscardedFrame");
omnetpp::simsignal_t ProtocolModuleBase::droppedPk = registerSignal(
        "droppedPk");
omnetpp::simsignal_t ProtocolModuleBase::bufferedPk = registerSignal(
        "bufferedPk");
omnetpp::simsignal_t ProtocolModuleBase::bufferedFrame = registerSignal(
        "bufferedFrame");
omnetpp::simsignal_t ProtocolModuleBase::bufferingTime = registerSignal(
        "bufferingTime");
omnetpp::simsignal_t ProtocolModuleBase::bufferLength = registerSignal(
        "bufferLength");
constexpr char ProtocolModuleBase::FROM_APP_GATE_NAME[];
constexpr char ProtocolModuleBase::TO_APP_GATE_NAME[];
constexpr char ProtocolModuleBase::FROM_RADIO_GATE_NAME[];
constexpr char ProtocolModuleBase::TO_RADIO_GATE_NAME[];

int ProtocolModuleBase::numInitStages() const {
    return 3;
}

void ProtocolModuleBase::initialize(int stage) {
    if (stage == 0) {
        this->_nodeNo = this->par("nodeNo");
        this->_dropAlreadySeenFrames =
                this->par("dropAlreadySeenFrames").boolValue();
    } else if (stage == 2) {
        /*if (this->par("fakeSentRoutingTable").boolValue()) {
         this->sendRoutingTableToProtocol(
         ContactPlanManager::getInstance()->getRoutingTable());
         }*/
    }
}

ProtocolModuleBase::~ProtocolModuleBase() {
}

void ProtocolModuleBase::sendToApp(omnetpp::cMessage *message) {
    emit(sentPacketToHL, message);
    // using the same gate index as in the sender network node
    this->send(message, TO_APP_GATE_NAME);
}

void ProtocolModuleBase::sendToRadio(inet::Packet *frame) {
    emit(outgoingFrame, frame);
    this->send(frame, TO_RADIO_GATE_NAME);
}

void ProtocolModuleBase::handleMessage(omnetpp::cMessage *message) {
    if (message->arrivedOn(FROM_APP_GATE_NAME)) {
        // we got a new packet to be send out from the upper layer
        this->receivedFromApp(omnetpp::check_and_cast<inet::Packet*>(message));
    } else if (message->arrivedOn(FROM_RADIO_GATE_NAME)) {
        // we got a new frame to be handled from the lower layer
        this->receivedFromRadio(
                omnetpp::check_and_cast<inet::Packet*>(message));
    } else if (contains(this->_bufferedTimeouts, message)) {
        // we got a buffer timeout, we'll wait for the packet to arrive from the protocol buffer
        this->handleBufferTimeout(message);
    } else {
        throw omnetpp::cRuntimeError("Unexpected message received");
    }
}

void ProtocolModuleBase::handleBufferTimeout(omnetpp::cMessage *message) {
    auto bufIt = std::find(this->_bufferedTimeouts.begin(),
            this->_bufferedTimeouts.end(), message);
    this->_bufferedTimeouts.erase(bufIt);
    this->cancelAndDelete(message);
    this->sendTimeToProtocol();
    this->waitForProtocolResponse(nullptr);
    // buffer statistics collection
    emit(bufferLength, (unsigned long int) this->_bufferedTimeouts.size());
}

void ProtocolModuleBase::receivedFromApp(inet::Packet *appPkt) {
    emit(rcvdPacketFromHL, appPkt);
    // let the protocol know and work
    this->processPacketFromUpperLayer(appPkt);
    this->waitForProtocolResponse(appPkt);
}

void ProtocolModuleBase::receivedFromRadio(inet::Packet *radioFrame) {
    emit(incomingFrame, radioFrame);

    // add tag to count hops if not there and otherwise up it by one
    // only do it as we receive packets from the radio as this should correspond to the number of hops
    auto numHopsHeaderOld = radioFrame->popAtBack<NumHopsHeader>(inet::B(1));
    auto numHopsHeader = inet::makeShared<NumHopsHeader>();
    numHopsHeader->setNumHops(numHopsHeaderOld->getNumHops() + 1);
    radioFrame->trimBack();
    radioFrame->insertAtBack(numHopsHeader);

    // let the protocol know and work
    this->processPacketFromLowerLayer(radioFrame);
    this->waitForProtocolResponse(radioFrame);
}

void ProtocolModuleBase::waitForProtocolResponse(
        omnetpp::cMessage *argMessage) {
    // TODO not do busy waiting
    while (!this->dataFromProtocol()) {
        sleep(1);
    }
    std::string commandStringResponse;
    omnetpp::cMessage *message = this->recvFromProtocol();
    if (message->getKind() == PROTOCOL_MODULE_PACKET_THROWAWAY) {
        emit(droppedPk, argMessage);
        // argMessage will be dropped by our caller, so we're just deleting the message
        delete message;
        return;
    } else if (message->getKind() == PROTOCOL_MODULE_BUFFER_TIMEOUT) {
        // schedule the message for later, when the packet will be removed from the buffer
        omnetpp::SimTime timeout = message->getTimestamp();
        message->setTimestamp();
        this->_bufferedTimeouts.push_back(message);
        this->scheduleAt(timeout, message);
        // buffer statistics collection
        emit(bufferLength, (unsigned long int) this->_bufferedTimeouts.size());
        emit(bufferingTime, timeout - omnetpp::simTime());

        /*inet::Packet *appPkt = dynamic_cast<inet::Packet*>(message);
         auto numHop = appPkt->findTag<NumHopsTag>();
         if (numHop != nullptr) {
         emit(bufferedFrame, appPkt);
         } else {
         emit(bufferedPk, appPkt);
         }* TODO*/

        return;
    }
    inet::Packet *appPkt = dynamic_cast<inet::Packet*>(message);
    auto nextHop = appPkt->findTag<NextHopReq>();
    if (nextHop != nullptr) {
        this->receivedRadioFrameFromProtocol(appPkt);
    } else if (appPkt != nullptr) {
        this->receivedFromProtocol(appPkt);
    } else {
        throw omnetpp::cRuntimeError("Unknown message type");
    }
}

void ProtocolModuleBase::receivedRadioFrameFromProtocol(
        inet::Packet *radioFrame) {
    this->sendToRadio(radioFrame);
}

void ProtocolModuleBase::receivedFromProtocol(inet::Packet *appPkt) {
    this->sendToApp(appPkt);
}

}  // namespace estnet
