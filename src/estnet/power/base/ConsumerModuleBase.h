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

#ifndef ENERGYMODELINET_CONSUMER_CONSUMERMODULES_CONSUMERMODULEBASE_H_
#define ENERGYMODELINET_CONSUMER_CONSUMERMODULES_CONSUMERMODULEBASE_H_

#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/power/contract/IEpEnergySource.h>
#include <inet/common/Units.h>

#include "estnet/common/ESTNETDefs.h"
#include "estnet/power/base/ConsumerStateHandler.h"

using namespace omnetpp;

namespace estnet {

class ESTNET_API ConsumerStateHandler;

typedef inet::units::value<double, inet::units::units::W> W;

/**
 *  Base class for all consumer modules
 *  has list of stateHandler which are asked about the current consumption
 *  -> chooses max. value
 *  Additionally, the class contains the consumer, that are coupled on this
 *  module, as this is required to inform those modules about state changes
 */
class ESTNET_API ConsumerModuleBase: public cModule,
        public inet::power::IEpEnergyConsumer {
protected:
    /** @brief returns number of initialization stages
     *  @return int: number of initialization stages */
    virtual int numInitStages() const override;

    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

    /** @brief sends status information to consumers, which are coupled to this module
     *  @param on: activation status that is sent to all coupled consumer */
    virtual void sendStatusToCoupledConsumers(bool on);

    /** list of all state handlers, that decide about the consumers state */
    std::vector<ConsumerStateHandler*> _stateHandler;

    /** list of all consumers, which are coupled to this module */
    std::vector<ConsumerStateHandler*> _coupledConsumer;

    /** stores value of power consumption */
    W _consumption;

    /** ptr to save energy source */
    inet::power::IEpEnergySource *_energySource = nullptr;

public:
    /** @brief returns module, which is the energy source for this consumer
     *  @return IEnergySource: battery that is used by the consumer*/
    virtual inet::power::IEnergySource* getEnergySource() const override;

    /** @brief returns power consumed at the moment
     *  @return W: current power consumption in Watt */
    virtual W getPowerConsumption() const override;

    /** @brief add state handler to consumer module
     *  @param stateHandler: state handler that is added to consumer module */
    virtual void addStateHandler(ConsumerStateHandler *stateHandler);

    /** @brief update consumption depending on the proposed consumption of all state handlers */
    virtual void powerConsumptionChanged();

    /** @brief add another coupled consumer module, has to be CoupledConsumer
     *  @param consumer: state handler that is coupled to consumer module */
    virtual void addCoupledConsumer(ConsumerStateHandler *consumer);

};

}  // namespace estnet

#endif /* ENERGYMODELINET_CONSUMER_CONSUMERMODULES_CONSUMERMODULEBASE_H_ */
