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

#ifndef NODES_ERRORMODEL_H_
#define NODES_ERRORMODEL_H_

#include <inet/physicallayer/contract/packetlevel/IRadio.h>

#include "estnet/node/satellite/Satellite.h"
#include "estnet/siminterface/pubsub/SimplePublisher.h"

using namespace omnetpp;
using namespace inet;
using namespace physicallayer;

namespace estnet {

/*
 * implements failure simulation
 * handles failure due to jamming stations set or radiation
 */
class ESTNET_API NodeFailureModel: public estnet::SimplePublisher {
protected:
    /** @brief initialization of module, checks if errormodel is enabled an sends start messages */
    virtual void initialize();
    /** @brief handles the messages of the errorModel module:
     * the failure/reparation of the node */
    virtual void handleMessage(cMessage *msg);

private:
    cMessage *_failure;
    cMessage *_repaired;
    cMessage *_check_timer;
    double _MTTF, _MTTR;
    int _checkIntervall;
    int _faultSeed;
    Satellite *_node;
    std::string _msgKey;
    static omnetpp::simsignal_t nodeFailed;

};

}  // namespace estnet

#endif /* NODES_ERRORMODEL_H_ */
