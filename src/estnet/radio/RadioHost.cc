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

#include "RadioHost.h"

#include <inet/linklayer/common/MacAddressTag_m.h>
#include <inet/common/ProtocolTag_m.h>
#include <inet/linklayer/common/MacAddressTag_m.h>
#include <inet/common/packet/tag/TagSet.h>

#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/common/StlUtils.h"
#include "estnet/common/AddressUtils.h"
#include "estnet/common/ModuleAccess.h"
#include "estnet/protocol/common/NextHopTag_m.h"

namespace estnet {

Define_Module(RadioHost)

int RadioHost::numInitStages() const {
    return 3;
}

void RadioHost::initialize(int stage) {
    if (stage == 0) {
        this->_nodeRegistry = NodeRegistry::getInstance();
        this->_nodeNo = this->par("nodeNo");

        _lastSendTime = -1;
        _msgSameTime = 0;

    } else if (stage == 2) {
        this->_gs = this->_nodeRegistry->getGroundStation(this->_nodeNo);

        // find all connected radios
        for (int i = 0; i < this->gateSize("lowerLayerOut"); i++) {
            omnetpp::cGate *startGate = this->gate("lowerLayerOut", i);
            if (startGate->getNextGate()->getOwnerModule()->hasGate(
                    "lowerLayerOut")) {
                startGate = startGate->getNextGate()->getOwnerModule()->gate(
                        "lowerLayerOut");
            }
            for (omnetpp::cGate *gate = startGate->getNextGate(); gate; gate =
                    gate->getNextGate()) {
                cModule *nic = findContainingNicModule(gate->getOwnerModule());
                if (nic != nullptr) {
                    inet::physicallayer::IRadio *radio =
                            omnetpp::check_and_cast<inet::physicallayer::IRadio*>(
                                    nic->getSubmodule("radio"));
                    this->_connectedRadios.emplace(i, radio);
                    // found nic, we can stop walking along the gates
                    break;
                }
            }
        }
    }
}

void RadioHost::handleMessage(omnetpp::cMessage *msg) {
    if (msg->arrivedOn("upperLayerIn")) {
        // message from the upper layer
        // we'll have to decide if we use the radio or send it directly to the
        // ground station node
        inet::Packet *pkt = dynamic_cast<inet::Packet*>(msg);
        auto nextHopTag = pkt->removeTag<NextHopReq>();
        long nextHop = nextHopTag->getNextHopNodeNo();

        bool broadcast = nextHop == 0;
        unsigned int destNodeId = nextHop;

        auto macAddressReq = pkt->addTagIfAbsent<inet::MacAddressReq>();
        if (broadcast) {
            macAddressReq->setDestAddress(inet::MacAddress::BROADCAST_ADDRESS);
        } else {
            macAddressReq->setDestAddress(getMacAddressOfNode(destNodeId, 1));
        }
        macAddressReq->setSrcAddress(getMacAddressOfNode(this->_nodeNo, 1));

        GroundStation *destinationCGS = this->isForGroundStation(destNodeId);

        // FIXME: As of inet 4.2 the acking mac module can not handle two or more
        // packages scheduled at exactly the same time. So the following packets
        // scheduled at exactly the same time are delayed. The delays are chosen
        // to account for the internal delay of the mac module. It should not effect
        // the simulation results, since the time delay is small compared to a usual
        // transmission duration.
        if (fabs(_lastSendTime - omnetpp::simTime()) <= 2e-6) {
            _msgSameTime = 1;
        } else {
            _msgSameTime = 0;
            _lastSendTime = omnetpp::simTime();
        }

        if (destinationCGS != nullptr) {
            // packet is for one specific ground station
            EV_DEBUG << "Sending frame '" << pkt->getName()
                            << "' to connected ground station" << omnetpp::endl;
            this->sendToGroundStation(destinationCGS, pkt);
            // we wasted a packet request from the radio, so we'll re-request
            //this->_queueModule->requestPacket();
        } else if (destinationCGS == nullptr && !broadcast) {
            // wasn't for a specific ground station => sent out via radio
            int radioGateIdx = this->chooseRadio(destNodeId);
            EV_DEBUG << "Sending frame '" << pkt->getName() << "' to radio "
                            << radioGateIdx << omnetpp::endl;
            this->sendDirect(pkt, _msgSameTime * 2e-6, 0, this, "lowerLayerOut",
                    radioGateIdx);
        } else if (broadcast) {
            EV_DEBUG << "Broadcasting frame '" << pkt->getName()
                            << "' to all connected ground stations and radios"
                            << omnetpp::endl;
            // is a broadcast, first sent to all other ground stations
            for (const auto &otherGs : this->_nodeRegistry->getGroundStations()) {
                if (this->_nodeNo != otherGs->getNodeNo())
                    this->sendToGroundStation(otherGs, pkt->dup());
            }
            // then send to all radios
            const int numLowerLayerOut = this->gateSize("lowerLayerOut");
            const double interTxDelay = (numLowerLayerOut > 1) ? 0.1 : 0.0; //FIXME: find more suitable solution
            for (int i = 0; i < numLowerLayerOut; i++) {
                // using a small delay, so broadcasts don't interfer with each when nodes are moving
                this->sendDirect(pkt->dup(), interTxDelay + _msgSameTime * 2e-6,
                        0, this, "lowerLayerOut", i);
            }
            // delete original packet since we duplicated it
            delete pkt;
        }
    } else if (msg->arrivedOn("lowerLayerIn")) {
        // we just forward to the upper layer
        this->sendDirect(msg, this, "forwardToUpperLayer");
    } else {
        throw omnetpp::cRuntimeError("arrived on unknown gate");
    }
}

bool RadioHost::isBroadcast(const inet::MacAddress &macAddress) {
    return macAddress == inet::MacAddress::BROADCAST_ADDRESS;
}

GroundStation* RadioHost::isForGroundStation(unsigned int destNodeId) {
    GroundStation *otherGs = this->_nodeRegistry->getGroundStation(destNodeId);
    EV_TRACE << "RadioHost::isForGroundStation(" << destNodeId << ") thisGs"
                    << (this->_gs != nullptr) << ", otherGs "
                    << (otherGs != nullptr) << "" << omnetpp::endl;
    if (this->_gs != nullptr && otherGs != nullptr
            && this->_gs->canCommunicateWithoutRadioWith(destNodeId)) {
        return otherGs;
    } else {
        return nullptr;
    }
}

void RadioHost::sendToGroundStation(GroundStation *otherGs, inet::Packet *pkt) {
    this->sendDirect(pkt,
            otherGs->getSubmodule("networkHost")->getSubmodule("radioHost"),
            "forwardToUpperLayer");
}

int RadioHost::chooseRadio(unsigned int destNodeId) {
    // get access to target node and its position
    int bestGateIdx = -1;
    double bestGateMetric = std::numeric_limits<double>::infinity();
    NodeBase *destinationNode = this->_nodeRegistry->getNode(destNodeId);
    inet::IMobility *destMobility = destinationNode->getMobility();
    inet::Coord destPosition = destMobility->getCurrentPosition();

    // if the node has multiple radios find the one with smallest antenna pointing error
    if (this->_connectedRadios.size() > 1) {
        for (const auto &connectedRadioEntry : this->_connectedRadios) {
            int currentGateIdx = connectedRadioEntry.first;
            inet::physicallayer::IRadio *currentRadio =
                    connectedRadioEntry.second;
            inet::IMobility *antennaMobility =
                    currentRadio->getAntenna()->getMobility();
            inet::Coord antennaPosition = antennaMobility->getCurrentPosition();
            inet::Quaternion antennaOrientation =
                    antennaMobility->getCurrentAngularPosition();

            inet::Quaternion antennaOrientationConj = antennaOrientation;
            antennaOrientationConj.conjugate();

            inet::Coord connectingVector = antennaPosition - destPosition;
            connectingVector.normalize();
            antennaOrientationConj.rotate(connectingVector);

            //Assumption: Antenna is pointing in X_AXIS for RPY=(0,0,0)
            inet::Quaternion orientation = inet::Quaternion::rotationFromTo(
                    inet::Coord::X_AXIS, connectingVector);
            orientation.normalize();
            double product = std::min(1.0f,
                    std::max(-1.0f,
                            orientation.rotate(inet::Coord::X_AXIS)
                                    * inet::Coord::X_AXIS));
            double totalAngles = acos(product);

            EV_TRACE << "Radio " << currentGateIdx << " angle: " << totalAngles
                            << omnetpp::endl;

            if (totalAngles < bestGateMetric) {
                bestGateIdx = currentGateIdx;
                bestGateMetric = totalAngles;
            }
        }
    } else if (this->_connectedRadios.size() == 1) {
        auto it = this->_connectedRadios.begin();
        bestGateIdx = it->first;
    }

    if (bestGateIdx == -1) {
        throw omnetpp::cRuntimeError("Could not choose radio");
    }
    EV_DEBUG << "Choosing radio " << bestGateIdx << omnetpp::endl;
    return bestGateIdx;
}

} // namespace estnet
