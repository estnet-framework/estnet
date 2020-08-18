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

#include "JammedPacketHandler.h"

#include <inet/physicallayer/base/packetlevel/NarrowbandReceiverBase.h>

namespace estnet {

Define_Module(JammedPacketHandler);

omnetpp::simsignal_t JammedPacketHandler::jammedPacketCount = registerSignal(
        "jammedPacketCount");
omnetpp::simsignal_t JammedPacketHandler::lostPacketCount = registerSignal(
        "lostPacketCount");

JammedPacketHandler::JammedPacketHandler() :
        Subscriber(this->getModuleByPath(".^.^.")) {
}

void JammedPacketHandler::initialize(int stage) {
    if (stage == 10) {
        //initialize all the class members
        this->_node = check_and_cast<NodeBase*>(
                this->getParentModule()->getParentModule());
        this->_nodeFailureState = false;
        // get coupled radio
        auto nic = this->getParentModule()->getSubmodule("wlan",
                this->par("radioIndex").intValue());
        this->_radio = check_and_cast<inet::physicallayer::IRadio*>(
                nic->getSubmodule("radio"));
        const inet::physicallayer::NarrowbandReceiverBase *rx;
        try {
            rx =
                    dynamic_cast<const inet::physicallayer::NarrowbandReceiverBase*>(_radio->getReceiver());

        } catch (std::exception &e) {
            rx = nullptr;
        }

        //store all possible jamming stations
        auto rootModule = this->_node->getParentModule();
        int num = rootModule->par("numJammer");
        for (int i = 0; i < num; i++) {
            auto jammer = check_and_cast<JammingStation*>(
                    rootModule->getSubmodule("jammer", i));
            if (rx != nullptr) {
                if ((rx->getCenterFrequency() + rx->getBandwidth() / 2)
                        > (jammer->getCenterFrequency()
                                + jammer->getBandwidth() / 2)
                        && (rx->getCenterFrequency() - rx->getBandwidth() / 2)
                                > (jammer->getCenterFrequency()
                                        + jammer->getBandwidth() / 2)) {
                    continue;
                } else if ((rx->getCenterFrequency() + rx->getBandwidth() / 2)
                        < (jammer->getCenterFrequency()
                                - jammer->getBandwidth() / 2)
                        && (rx->getCenterFrequency() - rx->getBandwidth() / 2)
                                < (jammer->getCenterFrequency()
                                        - jammer->getBandwidth() / 2)) {
                    continue;
                }
            }
            this->_jammers.push_back(jammer);
        }

        // set counter to zero and emit the initial value
        this->_lostPacketCounter = 0;
        this->_jammedPacketCounter = 0;
        this->emit(jammedPacketCount, this->_jammedPacketCounter);
        this->emit(lostPacketCount, this->_lostPacketCounter);

        // subscribe node failure state
        subscribeTopic("/omnet/satellite/nodefailure/");
    }
}

void JammedPacketHandler::handleMessage(cMessage *msg) {

    if (msg->arrivedOn("lowerLayerIn") || msg->arrivedOn("upperLayerIn")) {
        //check whether node is jammed by a jammingStation
        inet::Coord position = this->_node->getMobility()->getCurrentPosition();
        bool isJammed = false;
        for (JammingStation *jammer : this->_jammers) {
            if (jammer->isInJammingRange(position)) {
                if (uniform(0, 1) < jammer->getProbability()) {
                    isJammed = true;
                }
            }
        }
        if (isJammed && !msg->arrivedOn("upperLayerIn")) {
            // node got jammed
            this->_jammedPacketCounter++;
            this->emit(jammedPacketCount, this->_jammedPacketCounter);
            EV_INFO << "Node got jammed: Packet is lost" << omnetpp::endl;
        } else if (_nodeFailureState) {
            // node is in failure state
            this->_lostPacketCounter++;
            this->emit(lostPacketCount, this->_lostPacketCounter);
            EV_INFO << "Node is in failure state: Packet is lost"
                           << omnetpp::endl;
        } else {
            // forward packet
            if (msg->arrivedOn("upperLayerIn")) {
                this->send(msg, this->gate("lowerLayerOut"));
            } else {
                this->send(msg, this->gate("upperLayerOut"));
            }
        }

    } else {
        throw omnetpp::cRuntimeError("Message arrived on unknown gate");
    }
}

void JammedPacketHandler::receivedPubSubMessage(estnet::PubSubMsg *pubSubMsg) {
    if ((std::string) pubSubMsg->key == "/omnet/satellite/nodefailure/") {
        _nodeFailureState = atoi((pubSubMsg->value).c_str());
        EV << "Satellites failure state is " << _nodeFailureState << std::endl;
    }
}

void JammedPacketHandler::finish() {
    if (!this->_jammers.empty()) {
        EV_INFO << "Number of jammed packets: " << this->_jammedPacketCounter
                       << omnetpp::endl;
    }
    EV_INFO << "Number of lost packets due to node failure: "
                   << this->_lostPacketCounter << omnetpp::endl;
}

} //namespace
