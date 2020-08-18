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

#include "estnet/node/base/NodeBase.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"
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
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
    virtual void receivedPubSubMessage(estnet::PubSubMsg *pubSubMsg);

private:
    std::vector<JammingStation*> _jammers;
    NodeBase *_node;
    inet::physicallayer::IRadio *_radio;
    int _lostPacketCounter;
    int _jammedPacketCounter;
    bool _nodeFailureState;             ///< true if there is a node failure

    static omnetpp::simsignal_t jammedPacketCount;
    static omnetpp::simsignal_t lostPacketCount;
};

} //namespace

#endif
