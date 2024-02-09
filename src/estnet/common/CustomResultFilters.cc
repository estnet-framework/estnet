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

#include "CustomResultFilters.h"

#include <inet/linklayer/acking/AckingMacHeader_m.h>
#include <inet/physicallayer/unitdisk/UnitDiskPhyHeader_m.h>
#include <inet/common/packet/chunk/Chunk.h>
#include <inet/common/packet/Packet.h>
#include <inet/common/Simsignals_m.h>
#include <inet/linklayer/acking/AckingMac.h>
#include <inet/common/packet/chunk/Chunk.h>

#include "estnet/protocol/common/NextHopTag_m.h"
#include "estnet/protocol/common/NumHopsHeader_m.h"
#include "estnet/application/common/AppHeader_m.h"
#include "estnet/application/common/AppHostHeader_m.h"
#include "estnet/application/common/DestNodeIdTag_m.h"
#include "estnet/application/common/SrcNodeIdTag_m.h"
#include "estnet/node/base/NodeBase.h"
#include "estnet/common/AddressUtils.h"

namespace estnet {
using namespace inet;

void OneFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    fire(this, t, 1ul, details);
}
Register_ResultFilter("constantOne", OneFilter);

void MinusOneFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    fire(this, t, -1l, details);
}
Register_ResultFilter("constantMinusOne", MinusOneFilter);

void NegativePacketBytesFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto msg = dynamic_cast<omnetpp::cPacket*>(object))
        fire(this, t, -(double) msg->getByteLength(), details);
}
Register_ResultFilter("negativePacketBytes", NegativePacketBytesFilter);

void PacketNumHopsFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto packet = dynamic_cast<inet::Packet *>(object)) {
        auto numHopsHeader = packet->peekAtBack<NumHopsHeader>(inet::B(1));
        fire(this, t, numHopsHeader->getNumHops(), details);
    }
}
Register_ResultFilter("pktNumHops", PacketNumHopsFilter);

void PacketSequenceNumberFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto packet = dynamic_cast<inet::Packet *>(object)) {
        auto appHeader = packet->peekAtFront<AppHeader>();
        fire(this, t, (unsigned long) appHeader->getPktId(), details);
    }
}
Register_ResultFilter("pktSequenceNumber", PacketSequenceNumberFilter);

void PacketSourceNodeNoFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto msg = dynamic_cast<Packet*>(object)) {
        if (msg->hasAtFront<AppHostHeader>()) {
            auto appHostHeader = msg->peekAtFront<AppHostHeader>();
            fire(this, t, (unsigned long) appHostHeader->getSourceNodeID(),
                    details);
        }
    }
}
Register_ResultFilter("pktSrcNodeNo", PacketSourceNodeNoFilter);

void PacketDestinationNodeNoFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto msg = dynamic_cast<Packet*>(object)) {
        if (msg->hasAtFront<AppHostHeader>()) {
            auto appHostHeader = msg->peekAtFront<AppHostHeader>();
            fire(this, t, (unsigned long) appHostHeader->getDestNodeID(),
                    details);
        }
    }
}
Register_ResultFilter("pktDestNodeNo", PacketDestinationNodeNoFilter);

void NextHopNodeNoFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto msg = dynamic_cast<Packet*>(object)) {
        try {
            auto nextHopTag = msg->getTag<NextHopReq>();
            fire(this, t, nextHopTag->getNextHopNodeNo(), details);
        } catch (std::exception& e) {

        }
    }
}
Register_ResultFilter("pktNextHopNodeNo", NextHopNodeNoFilter);

void MacFrameSourceNodeNoFilter::receiveSignal(omnetpp::cResultFilter *prev,
        omnetpp::simtime_t_cref t, omnetpp::cObject *object,
        omnetpp::cObject *details) {
    if (auto pkt = dynamic_cast<Packet*>(object)) {
        inet::Ptr<const inet::AckingMacHeader> ackingMacHeader;
        try{
            ackingMacHeader = pkt->peekAtFront<inet::AckingMacHeader>();
        } catch(std::exception& e){
            EV_ERROR << "Packet does not have peekable AckingMacHeader" << endl;
        }
        if (ackingMacHeader) {
            auto srcAddress = ackingMacHeader->getSrc();
            fire(this, t, (unsigned long) srcAddress.getInt(), details);
        }
    }
}
Register_ResultFilter("macFrameSrcNodeNo", MacFrameSourceNodeNoFilter);

void MacFrameDestinationNodeNoFilter::receiveSignal(
        omnetpp::cResultFilter *prev, omnetpp::simtime_t_cref t,
        omnetpp::cObject *object, omnetpp::cObject *details) {
    if (auto pkt = dynamic_cast<Packet*>(object)) {
        auto ackingMacHeader = pkt->peekAtFront<inet::AckingMacHeader>();
        if (ackingMacHeader) {
            auto dAddress = ackingMacHeader->getDest();
            fire(this, t, (unsigned long) dAddress.getInt(), details);
        }
    }
}
Register_ResultFilter("macFrameDestNodeNo", MacFrameDestinationNodeNoFilter);


void PacketDroppedDueToBitErrorOrCollision::receiveSignal(
        omnetpp::cResultFilter *prev, omnetpp::simtime_t_cref t,
        omnetpp::cObject *object, omnetpp::cObject *details) {

    if (check_and_cast<PacketDropDetails *>(details)->getReason() == inet::INCORRECTLY_RECEIVED) {
        if (auto pkt = dynamic_cast<Packet*>(object)) {
            // get node number of the node where the packet arrived
            auto mac = dynamic_cast<inet::AckingMac*>(pkt->getArrivalModule());
            auto node = dynamic_cast<estnet::NodeBase *>(mac->getModuleByPath("^.^.^"));
            // get mac address of current node
            auto currNodeMacAddress = getMacAddressOfNode(node->getNodeNo(), 1);

            auto macHeader = pkt->peekAtFront<AckingMacHeader>();
            if (macHeader->getDest().equals(currNodeMacAddress)) {
                fire(this, t, object, details);
            }
        }
    }
}
Register_ResultFilter("packetDroppedDueToBitErrorOrCollision", PacketDroppedDueToBitErrorOrCollision);

}  // namespace estnet
