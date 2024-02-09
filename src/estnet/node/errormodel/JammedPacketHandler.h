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

#ifndef __ESTNET_JAMMEDPACKETHANDLER_H_
#define __ESTNET_JAMMEDPACKETHANDLER_H_

#include <inet/physicallayer/contract/packetlevel/IRadio.h>

#include "estnet/node/base/NodeBase.h"
#include "estnet/node/errormodel/JammingStation.h"
#include "estnet/siminterface/pubsub/Subscriber.h"

namespace estnet {

/**
 *  Module that passes packets only in non error state
 */
class ESTNET_API JammedPacketHandler: public cSimpleModule,
        public estnet::Subscriber {
public:
    JammedPacketHandler();
protected:
    /**
     * Multi-stage initialization hook
     * @return number of initialize stages
     */
    virtual int numInitStages() const {
        return 11;
    }
    /**
     * @brief Initialize the module in multiple stages
     */
    virtual void initialize(int stage);
    /**
     * @brief Handles incoming packets and throws them away,
     * if node is in error state or is jammed
     */
    virtual void handleMessage(cMessage *msg);
    /**
     * Print statistics and clean up
     */
    virtual void finish();
    /**
     * React to messages via the publisher subscriber system
     * These message contain updates to the node failure state
     */
    virtual void receivedPubSubMessage(estnet::PubSubMsg *pubSubMsg);

private:
    std::vector<JammingStation*> _jammers;  // list of all jammers in network
    NodeBase *_node;                        // parent node, belonging node for this module
    inet::physicallayer::IRadio *_radio;    // belonging radio for this module
    int _lostPacketCounter;                 // stats of lost packets due to node failure
    int _jammedPacketCounter;               // stats of lost packets due to jammer
    bool _nodeFailureState;                 // true if there is a node failure
    std::string _msgKey;                    // subscription message key

    static omnetpp::simsignal_t jammedPacketCount;
    static omnetpp::simsignal_t lostPacketCount;
};

} //namespace

#endif
