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

#ifndef __ESTNET_ENERGYMODELINET_CONSUMER_CONSUMERSTATE_CONSUMERSTATEHANDLER_H_
#define __ESTNET_ENERGYMODELINET_CONSUMER_CONSUMERSTATE_CONSUMERSTATEHANDLER_H_

#include <inet/common/Units.h>
#include <inet/common/InitStages.h>

#include "estnet/power/base/ConsumerModuleBase.h"
#include "estnet/common/ESTNETDefs.h"

using namespace omnetpp;

namespace estnet {

class ConsumerModuleBase;

typedef inet::units::value<double, inet::units::units::W> W;

/**
 *  Base class for all consumer types, which supplies basic functions,
 *  but no logical component of simulating any consumption
 *  A consumer type is used in the consumer modules to simulate different
 *  power consumption behaviors
 *  To implement a new state handler, kind of a logic has to be implemented
 *  that sets the values [_powerConsumption] and [_on].
 *  This can be done for example by overriding the cSimpleModule function:
 *
 *      virtual void handleMessage(cMessage *message)
 *
 *  For consumer type examples please refer to:
 *  A. Freimann, J. Scharnagl, T. Petermann, K. Schilling;
 *  CubeSat Energy Modelling for Improved Mission Planning and Operations
 *
 */
class ESTNET_API ConsumerStateHandler: public cSimpleModule {
protected:
    /** ptr to energy consumer module */
    ConsumerModuleBase *_energyConsumer;

    /** current state */
    bool _on;

    /** power consumption in current state */
    W _powerConsumption;

    /** @brief initialization
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

    /** @brief returns number of initialization stages
     *  @return int: number of initialization stages */
    virtual int numInitStages() const;

public:
    /** @brief check whether the system is in state on
     *  @return bool: true if system is active */
    virtual bool isActive() const;

    /** @brief returns proposed power consumption for consumer module
     *  @return W: current power consumption in Watt */
    virtual W getProposedPowerConsumption() const;
};

}  // namespace estnet

#endif
