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

#include "ExternalProtocolModuleBase.h"

#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <cppcodec/base64_default_rfc4648.hpp>

#include "estnet/common/StlUtils.h"
#include "estnet/application/common/AppHeader_m.h"
#include "estnet/application/common/AppHostHeader_m.h"
#include "estnet/protocol/common/NextHopTag_m.h"

namespace estnet {

omnetpp::cMessage* ExternalProtocolModuleBase::recvFromProtocol() {
    // receive from protocol
    std::string commandString;
    this->recvFromProtocol(commandString);
    std::cout << "Node " << this->_nodeNo
            << ": received command string from protocol '" << commandString
            << "' (" << commandString.length() << ")" << std::endl;
    // parse received command string
    rapidjson::Document d;
    d.Parse(commandString.c_str());
    if (std::string(d["command"].GetString()) == std::string("send")) {
        return this->getRadioFrame(d);
    } else if (std::string(d["command"].GetString())
            == std::string("receive")) {
        return this->getAppPacket(d);
    } else if (std::string(d["command"].GetString())
            == std::string("receiveRoutingTable")) {
        return this->getAppPacket(d);
    } else if (std::string(d["command"].GetString())
            == std::string("buffered")) {
        return this->getTimeoutMessage(d);
    } else {
        throw omnetpp::cRuntimeError("Unknown command received");
    }
}

void ExternalProtocolModuleBase::sendTimeToProtocol() {
    std::string commandString;
    this->buildTimeCommand(std::string("timeSync"), commandString);
    this->sendToProtocol(commandString);
}

/*
 void ExternalProtocolModuleBase::sendRoutingTableToProtocol(
 const RoutingTable& routingTable) {
 long sequenceNumber = 0;
 // TODO : not hardcoded
 unsigned int maxPacketSize = 200;
 unsigned int numChunks = routingTable.getNumChunks(maxPacketSize);
 for (unsigned int currentChunk = 0; currentChunk < numChunks;
 currentChunk++) {
 uint8_t *byteStream;
 unsigned int byteStreamLen = routingTable.asByteStream(&byteStream,
 sequenceNumber, currentChunk, maxPacketSize);
 if (byteStreamLen > 0) {
 std::string commandString;
 this->buildReceiveCommand("receiveFRoutingTable", sequenceNumber, 0,
 0, this->_nodeNo, this->_nodeNo, (uint8_t*) byteStream,
 byteStreamLen, commandString);
 this->sendToProtocol(commandString);
 delete[] byteStream;
 }
 }
 }
 */

void ExternalProtocolModuleBase::processPacketFromUpperLayer(
        inet::Packet *appPacket) {
    // get required headers
    auto appHeader = appPacket->popAtFront<AppHeader>();
    auto appHostHeader = appPacket->popAtFront<AppHostHeader>();

    uint64_t sequenceNumber = appHeader->getPktId();
    unsigned int sourceAppId = appHeader->getSourceAppID();
    unsigned int destAppId = appHeader->getDestAppID();
    unsigned int sourceNodeNo = appHostHeader->getSourceNodeID();
    unsigned int destNodeNo = appHostHeader->getDestNodeID();

    std::string command("send");
    /*if (appPacket->getKind() == BASIC_ROUTING_TABLE_APP_PACKET_TYPE) {
     command = std::string("sendRoutingTable");
     }*/
    // send to protocol
    std::string commandString;
    this->buildSendCommand(command, sequenceNumber, sourceAppId, destAppId,
            sourceNodeNo, destNodeNo,
            &appPacket->peekDataAsBytes()->getBytes()[0],
            appPacket->peekDataAsBytes()->getBytes().size(), commandString);
    this->sendToProtocol(commandString);

    // save for later
    AppAddress seq = std::tie(sourceNodeNo, sourceAppId);
    SequenceNumber key = std::tie(seq, sequenceNumber);
    appPacket->trimFront();
    appPacket->insertAtFront(appHostHeader);
    appPacket->insertAtFront(appHeader);
    this->_appPackets[key] = appPacket;
}

void ExternalProtocolModuleBase::processPacketFromLowerLayer(
        inet::Packet *radioFrame) {
    // get required headers
    auto appHostHeader = radioFrame->popAtFront<AppHostHeader>();
    auto appHeader = radioFrame->popAtFront<AppHeader>();

    //save packet information
    uint64_t sequenceNumber = appHeader->getPktId();
    unsigned int sourceAppId = appHeader->getSourceAppID();
    unsigned int destAppId = appHeader->getDestAppID();
    unsigned int sourceNodeNo = appHostHeader->getSourceNodeID();
    unsigned int destNodeNo = appHostHeader->getDestNodeID();

    // send to protocol
    std::string commandString;
    this->buildReceiveCommand("receive", sequenceNumber, sourceAppId, destAppId,
            sourceNodeNo, destNodeNo,
            &radioFrame->peekDataAsBytes()->getBytes()[0],
            radioFrame->peekDataAsBytes()->getBytes().size(), commandString);
    this->sendToProtocol(commandString);

    // save for later
    AppAddress seq = std::tie(sourceNodeNo, sourceAppId);
    SequenceNumber key = std::tie(seq, sequenceNumber);
    radioFrame->trimFront();
    radioFrame->insertAtFront(appHostHeader);
    radioFrame->insertAtFront(appHeader);
    this->_radioFrames[key] = radioFrame;
}

void ExternalProtocolModuleBase::buildTimeCommand(const std::string &command,
        std::string &commandString) {

    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> jsonWriter(s);
    jsonWriter.StartObject();
    jsonWriter.Key("command");
    jsonWriter.String(command.c_str());
    jsonWriter.Key("time");
    jsonWriter.Uint64(omnetpp::simTime().inUnit(omnetpp::SIMTIME_S));
    jsonWriter.Key("sequenceNumber");
    jsonWriter.Uint64(0);
    jsonWriter.Key("sourceAppId");
    jsonWriter.Uint(0);
    jsonWriter.Key("destAppId");
    jsonWriter.Uint(0);
    jsonWriter.Key("sourceNodeNo");
    jsonWriter.Uint(0);
    jsonWriter.Key("destinationNodeNo");
    jsonWriter.Uint(0);
    jsonWriter.Key("payloadSize");
    jsonWriter.Uint64(0);
    jsonWriter.Key("payload");
    jsonWriter.String("");
    jsonWriter.EndObject();

    commandString = s.GetString();
    std::cout << "Node " << this->_nodeNo
            << ": sending command string to protocol '" << commandString
            << "' (" << commandString.length() << ")" << std::endl;
}

void ExternalProtocolModuleBase::buildSendCommand(const std::string &command,
        uint64_t sequenceNumber, unsigned int sourceAppId,
        unsigned int destAppId, unsigned int sourceNodeNo,
        unsigned int destinationNodeNo, const uint8_t *payload,
        size_t payloadSize, std::string &commandString) {

    std::string encodedPayload = base64::encode(payload, payloadSize);
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> jsonWriter(s);
    jsonWriter.StartObject();
    jsonWriter.Key("command");
    jsonWriter.String(command.c_str());
    jsonWriter.Key("time");
    jsonWriter.Uint64(omnetpp::simTime().inUnit(omnetpp::SIMTIME_S));
    jsonWriter.Key("sequenceNumber");
    jsonWriter.Uint64(sequenceNumber);
    jsonWriter.Key("sourceAppId");
    jsonWriter.Uint(sourceAppId);
    jsonWriter.Key("destAppId");
    jsonWriter.Uint(destAppId);
    jsonWriter.Key("sourceNodeNo");
    jsonWriter.Uint(sourceNodeNo);
    jsonWriter.Key("destinationNodeNo");
    jsonWriter.Uint(destinationNodeNo);
    jsonWriter.Key("payloadSize");
    jsonWriter.Uint64(encodedPayload.length());
    jsonWriter.Key("payload");
    jsonWriter.String(encodedPayload.c_str());
    jsonWriter.EndObject();

    commandString = s.GetString();
    std::cout << "Node " << this->_nodeNo
            << ": sending command string to protocol '" << commandString
            << "' (" << commandString.length() << ")" << std::endl;
}

void ExternalProtocolModuleBase::buildReceiveCommand(const std::string &command,
        uint64_t sequenceNumber, unsigned int sourceAppId,
        unsigned int destAppId, unsigned int sourceNodeNo,
        unsigned int destinationNodeNo, const uint8_t *payload,
        size_t payloadSize, std::string &commandString) {

    std::string encodedPayload = base64::encode(payload, payloadSize);
    rapidjson::StringBuffer s;
    rapidjson::Writer<rapidjson::StringBuffer> jsonWriter(s);
    jsonWriter.StartObject();
    jsonWriter.Key("command");
    jsonWriter.String(command.c_str());
    jsonWriter.Key("time");
    jsonWriter.Uint64(omnetpp::simTime().inUnit(omnetpp::SIMTIME_S));
    jsonWriter.Key("sequenceNumber");
    jsonWriter.Uint64(sequenceNumber);
    jsonWriter.Key("sourceAppId");
    jsonWriter.Uint(sourceAppId);
    jsonWriter.Key("destAppId");
    jsonWriter.Uint(destAppId);
    jsonWriter.Key("sourceNodeNo");
    jsonWriter.Uint(sourceNodeNo);
    jsonWriter.Key("destinationNodeNo");
    jsonWriter.Uint(destinationNodeNo);
    jsonWriter.Key("payloadSize");
    jsonWriter.Uint64(encodedPayload.length());
    jsonWriter.Key("payload");
    jsonWriter.String(encodedPayload.c_str());
    jsonWriter.EndObject();

    commandString = s.GetString();
    std::cout << "Node " << this->_nodeNo
            << ": sending command string to protocol '" << commandString
            << "' (" << commandString.length() << ")" << std::endl;
}

omnetpp::cMessage* ExternalProtocolModuleBase::getTimeoutMessage(
        rapidjson::Document &d) {
    omnetpp::cMessage *bufferedTimeout = new omnetpp::cMessage(
            "bufferedTimeout", PROTOCOL_MODULE_BUFFER_TIMEOUT);
    bufferedTimeout->setTimestamp(omnetpp::SimTime(d["ut"].GetUint64()));
    return bufferedTimeout;
}

inet::Packet* ExternalProtocolModuleBase::getRadioFrame(
        rapidjson::Document &d) {

    uint64_t sequenceNumber = d["sequenceNumber"].GetUint64();
    unsigned int sourceAppId = d["sourceAppId"].GetUint();
    unsigned int sourceNodeNo = d["sourceNodeNo"].GetUint();
    AppAddress appAddr = std::tie(sourceNodeNo, sourceAppId);
    SequenceNumber key = std::tie(appAddr, sequenceNumber);

    unsigned int nextHopNodeNo = 0;
    if (d.HasMember("nextHopNodeNo")) {
        nextHopNodeNo = d["nextHopNodeNo"].GetUint();
    }

    // get the protocol's packet
    uint64_t payloadSize = d["payloadSize"].GetUint64();
    const char *payload = d["payload"].GetString();
    std::vector<uint8_t> protocolPacketBytes = base64::decode(payload,
            payloadSize);
    size_t protocolPacketSize = protocolPacketBytes.size();
    std::string protocolPacket(protocolPacketBytes.begin(),
            protocolPacketBytes.end());
    //TODO: what to do with this information?
    // rebuild the packet completly? -> packet content should not be changed by protocol

    if (this->_nodeNo == sourceNodeNo) {

        if (!contains(this->_appPackets, key)) {
            throw omnetpp::cRuntimeError(
                    "Did not find app packet for radio frame generation");
        }
        inet::Packet *appPacket = this->_appPackets.at(key);
        this->_appPackets.erase(key);
        // remove next hop tag to be sent to app
        appPacket->removeTagIfPresent<NextHopReq>();
        return appPacket;
    } else {
        // this is a packet we are forwarding, we should already have a radio frame
        // but since the protocol might change the packet, we will change next hop tag (or rebuild packet??)

        if (!contains(this->_appPackets, key)) {
            throw omnetpp::cRuntimeError(
                    "Did not find radio frame for forwarding");
        }
        inet::Packet *appPacket = this->_appPackets.at(key);
        this->_appPackets.erase(key);
        // update next hop
        auto nextHopTag = appPacket->addTagIfAbsent<NextHopReq>();
        nextHopTag->setNextHopNodeNo(nextHopNodeNo);
        return appPacket;
    }
}

inet::Packet* ExternalProtocolModuleBase::getAppPacket(rapidjson::Document &d) {
    // we're receiving an app packet, we must have a radio frame for it
    uint64_t sequenceNumber = d["sequenceNumber"].GetUint64();
    unsigned int sourceAppId = d["sourceAppId"].GetUint();
    unsigned int sourceNodeNo = d["sourceNodeNo"].GetUint();
    AppAddress appAddr = std::tie(sourceNodeNo, sourceAppId);
    SequenceNumber key = std::tie(appAddr, sequenceNumber);

    if (!contains(this->_appPackets, key)) {
        throw omnetpp::cRuntimeError(
                "Did not find radio frame for app packet delivery");
    }
    inet::Packet *radioFrame = this->_appPackets.at(key);
    this->_appPackets.erase(key);

    return radioFrame;
}

}  // namespace estnet
