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

#include <inet/physicallayer/contract/packetlevel/IRadio.h>

#include "BasicApp.h"

using namespace omnetpp;
using namespace inet;
using namespace physicallayer;

namespace estnet {

Define_Module(BasicApp);

omnetpp::simsignal_t BasicApp::sentPkSignal = registerSignal("sentPk");
omnetpp::simsignal_t BasicApp::rcvdPkSignal = registerSignal("rcvdPk");

int BasicApp::numInitStages() const {
    return 2;
}

void BasicApp::initialize(int stage) {
    if (stage == 0) {
        this->_startTime = par("startTime").doubleValue();
        this->_stopTime = par("stopTime").doubleValue();
        this->_scheduleMsg = new omnetpp::cMessage("appSchedule");
        this->_printNoPackets = par("printNoPackets").boolValue();
        this->_printReceivedPacketsNo =
                par("printReceivedPacketsNo").boolValue();
        this->_printSentPacketsNo = par("printSentPacketsNo").boolValue();
        this->_printMissingPacketsNo = par("printMissingPacketsNo").boolValue();
        this->_nodeId = par("nodeNo");
        this->_id = par("appId");
    } else if (stage == 1) {
        // schedule packet sending
        omnetpp::simtime_t nextTime;
        if (this->par("sending").boolValue()
                && this->nextPacketTime(nextTime)) {
            this->scheduleAt(nextTime, this->_scheduleMsg);
        }
    }
}

BasicApp::~BasicApp() {
    this->cancelAndDelete(this->_scheduleMsg);
}

void BasicApp::handleMessage(omnetpp::cMessage *msg) {
    if (msg == this->_scheduleMsg) {
        this->sendPacketInternal();
    } else {
        this->recvPacketInternal(msg);
        delete msg;
    }
}

short BasicApp::generatePacketKind() {
    return BASIC_APP_PACKET_TYPE;
}

void BasicApp::generatePacketDestinationNodeNumbers() {
    //reading parameter from ini file
    const char *nodeNoStr = par("destinationNodes").stringValue(); // e.g. "2 3 5";
    this->_destinationNodeNumbers =
            omnetpp::cStringTokenizer(nodeNoStr).asIntVector();
}

inet::Packet* BasicApp::generatePacketPayload() {
    // dummy packet without real payload data
    B payload = B(this->par("payloadSize").intValue());
    inet::Packet *newPacket = new inet::Packet();
    auto data = inet::makeShared<ByteCountChunk>(payload);
    newPacket->insertAtBack(data);
    return newPacket;
}

bool BasicApp::nextPacketTime(omnetpp::simtime_t &t) {
    double sendInterval = this->par("sendInterval").doubleValue();
    if (omnetpp::simTime() == 0.0) {
        t = this->_startTime + sendInterval;
    } else {
        t = omnetpp::simTime() + sendInterval;
    }
    if (sendInterval > 0
            && (this->_stopTime < SIMTIME_ZERO || t < this->_stopTime)) {
        return true;
    }
    return false;
}

void BasicApp::sendPacketInternal() {
    this->generatePacketDestinationNodeNumbers();

    for (auto destinationNodeNo : this->_destinationNodeNumbers) {
        // dummy payload
        inet::Packet *packet = this->generatePacketPayload();

        //add app header
        auto header = inet::makeShared<AppHeader>();
        header->setPktId(this->_numSent);
        header->setDestAppID(this->par("destAppId"));
        header->setSourceAppID(this->_id);
        packet->insertAtFront(header);

        //add tags
        auto destNodeIdTag = packet->addTagIfAbsent<DestNodeIdTag>();
        destNodeIdTag->setDestNodeId(destinationNodeNo);

        auto packetProtocolTag =
                packet->addTagIfAbsent<inet::PacketProtocolTag>();
        packetProtocolTag->setProtocol(&inet::Protocol::ipv4);

        //send packet to lower layer
        this->sendPacket(packet);

        // emit signal
        emit(sentPkSignal, packet);
        EV_INFO << "Sent packet  to " << destinationNodeNo << "with size "
                       << packet->getByteLength() << omnetpp::endl;

        // store packet informaion
        PacketInformation pktInfo;
        pktInfo.nodeId = getNodeId();
        pktInfo.appId = this->_id;
        pktInfo.pktId = this->_numSent;

        //auto sequenceNumber = node->getCurrentSequenceNumber();
        this->_packetsGenerated.push_back(pktInfo);
        this->_numSent++;
    }
    this->scheduleNextPacket();
}

void BasicApp::scheduleNextPacket(omnetpp::simtime_t nextTime) {
    // figure out when to send the next packet
    if (nextTime == -1) {
        if (this->nextPacketTime(nextTime)) {
            this->scheduleAt(nextTime, this->_scheduleMsg);
        }
    } else {
        this->scheduleAt(nextTime, this->_scheduleMsg);
    }
}

void BasicApp::sendPacket(inet::Packet *packet) {
    // actually send out packet to lower layer
    this->send(packet, APP_OUT_GATE_NAME);

}

void BasicApp::recvPacketInternal(omnetpp::cMessage *msg) {
    inet::Packet *pkt = this->recvPacket(msg);
    if (pkt != nullptr) {
        emit(rcvdPkSignal, pkt);
        //extracting headers and tags
        auto sourceNodeIdTag = pkt->getTag<SrcNodeIdTag>();
        auto numHopsHeader = pkt->peekAtBack<NumHopsHeader>(inet::B(1));
        auto appHeader = pkt->popAtFront<AppHeader>();

        //save packet information
        PacketInformation rcvdPacket;
        rcvdPacket.numHops = numHopsHeader->getNumHops();
        rcvdPacket.nodeId = sourceNodeIdTag->getSrcNodeId();
        rcvdPacket.appId = appHeader->getSourceAppID();
        rcvdPacket.pktId = appHeader->getPktId();

        if (this->_id != appHeader->getDestAppID()) {
            throw omnetpp::cRuntimeError(
                    "Apphost forwarded packet to wrong app!");
        }
        //print useful packet information
        EV_INFO << "Received packet " << rcvdPacket.pktId << " from node "
                       << rcvdPacket.nodeId << " and its app "
                       << rcvdPacket.appId << " with size "
                       << pkt->getByteLength() << " after "
                       << rcvdPacket.numHops << " hops." << omnetpp::endl;
        this->_numReceived++;

        this->_packetsReceived.push_back(rcvdPacket);
    }
}

inet::Packet* BasicApp::recvPacket(omnetpp::cMessage *msg) {
    inet::Packet *pkt = dynamic_cast<inet::Packet*>(msg);
    if (pkt == nullptr) {
        return nullptr;
    }
    return pkt;
}

void BasicApp::finish() {
    omnetpp::cSimpleModule::finish();

    if (this->_printNoPackets) {
        EV_INFO << "Packets sent: " << this->_numSent << omnetpp::endl;
        EV_INFO << "Packets received: " << this->_numReceived << omnetpp::endl;
    }

    if (this->_printSentPacketsNo) {
        EV_INFO << "Sent sequence numbers: ";
        std::stable_sort(this->_packetsGenerated.begin(),
                this->_packetsGenerated.end());
        for (const auto &pktInfo : this->_packetsGenerated) {
            EV_INFO << pktInfo.nodeId << "." << pktInfo.appId << "."
                           << pktInfo.pktId << ", ";
        }
        EV_INFO << omnetpp::endl;
    }

    if (this->_printReceivedPacketsNo) {
        EV_INFO << "Received packets with IDs: ";
        std::stable_sort(this->_packetsReceived.begin(),
                this->_packetsReceived.end());
        for (const auto &pktInfo : this->_packetsReceived) {
            EV_INFO << pktInfo.nodeId << "." << pktInfo.appId << "."
                           << pktInfo.pktId << "(" << pktInfo.numHops << "), ";

        }
        EV_INFO << omnetpp::endl;
    }

    if (this->_printMissingPacketsNo) {
        //EV_INFO << "Missing packets: " << omnetpp::endl;
        for (const auto &node : NodeRegistry::getInstance()->getNodes()) {
            auto apps = node->getApps();
            unsigned int nodeId = node->getNodeNo();
            if (nodeId == this->_nodeId) {
                continue;
            }
            EV_INFO << "Missing packets from Node" << nodeId << ": ";
            for (const auto app : apps) {
                unsigned int appId = app->getId();
                for (unsigned int i = 0; i < app->getPktSent(); i++) {
                    bool received = false;
                    for (const auto rPkt : this->_packetsReceived) {
                        if (rPkt.appId == appId && rPkt.nodeId == nodeId
                                && rPkt.pktId == i) {
                            received = true;
                            break;
                        }
                    }
                    if (!received) {
                        EV_INFO << nodeId << "." << appId << "." << i << ", ";
                    }
                }
            }
            EV_INFO << omnetpp::endl;

        }
        EV_INFO << omnetpp::endl;
    }
}

}  // namespace estnet
