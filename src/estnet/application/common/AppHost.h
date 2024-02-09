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

#ifndef APPS_APPHOST_H_
#define APPS_APPHOST_H_

#include <omnetpp.h>
#include <inet/common/packet/Packet.h>

#include "estnet/protocol/common/NumHopsHeader_m.h"
#include "AppHostHeader_m.h"
#include "AppHeader_m.h"
#include "DestNodeIdTag_m.h"
#include "SrcNodeIdTag_m.h"

namespace estnet {


/**
 * The app-host connects all different apps to the protocol
 * It adds node information header (source and destination Node ID).
 * Passing the received packet to the correct application.
 */
class ESTNET_API AppHost: public omnetpp::cSimpleModule {
private:
    long _numSent = 0;
protected:
    static constexpr char FROM_APP_GATE_NAME[] = "fromApp";
    static constexpr char TO_APP_GATE_NAME[] = "toApp";
    static constexpr char FROM_PROTOCOL_GATE_NAME[] = "lowerLayerIn";
    static constexpr char TO_PROTOCOL_GATE_NAME[] = "lowerLayerOut";

    static omnetpp::simsignal_t sentPkSignal;
    static omnetpp::simsignal_t rcvdPkSignal;
public:
    AppHost();

    /** @brief returns number of initalization stages */
    virtual int numInitStages() const override;
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief receives packets from radios or apps */
    virtual void handleMessage(omnetpp::cMessage *message) override;

    /** @brief called when the protocol returns a packet */
    virtual void receivedFromProtocolModule(inet::Packet *pkt);
    /** @brief called to deliver a packet to the protocol module */
    virtual void sendToProtocolModule(inet::Packet *pkt);
    /** @brief called when an app sends a packet */
    virtual void receivedFromApp(inet::Packet *pkt);
    /** @brief called to deliver a packet to an app */
    virtual void sendToApp(inet::Packet *pkt);

};

}  // namespace estnet

#endif /* APPS_APPHOST_H_ */
