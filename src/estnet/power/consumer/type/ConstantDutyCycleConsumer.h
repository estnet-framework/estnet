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

#ifndef ENERGYMODEL_CONSUMER_CONSTANTDUTYCYCLECONSUMER_H_
#define ENERGYMODEL_CONSUMER_CONSTANTDUTYCYCLECONSUMER_H_

#include <omnetpp.h>

#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/common/Units.h>

#include "estnet/power/base/ConsumerStateHandler.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::s> s;

/**
 *  Consumer class that has constant periods of consuming power.
 *  This power and the duty cycle stays constant.
 */
class ESTNET_API ConstantDutyCycleConsumer: public ConsumerStateHandler {
protected:
    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

    /** @brief scheduled self-messages receiver function
     *  @param  message: message that is received */
    virtual void handleMessage(cMessage *message) override;

    /** @brief change state and power consumption */
    virtual void updatePowerConsumption();

public:
    /** @brief cancel and delete scheduled messages */
    virtual ~ConstantDutyCycleConsumer();

private:
    W _consumption;      // power consumption in state on
    W _offConsumption;   // power consumption in state off
    s _onTime;           // time the node is in state on
    s _offTime;          // time the node is in state off
    bool _on;            // current state, true if on
    cMessage *_changeStatus;
};

}  // namespace estnet

#endif /* ENERGYMODEL_CONSUMER_CONSTANTDUTYCYCLECONSUMER_H_ */
