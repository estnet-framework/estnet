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

#ifndef ENERGYMODEL_CONSUMER_CONSUMERTYPES_COUPLEDCONSUMER_H_
#define ENERGYMODEL_CONSUMER_CONSUMERTYPES_COUPLEDCONSUMER_H_

#include <omnetpp.h>
#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/common/Units.h>

#include "estnet/power/base/ConsumerStateHandler.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::s> s;

/**
 *  Consumer class which status depends on another consumer.
 *  It has to register at this consumer module and gets a notification what status it should go to.
 */
class ESTNET_API CoupledConsumer: public ConsumerStateHandler {
protected:
    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

public:
    /** @brief cancel and delete scheduled messages */
    virtual ~CoupledConsumer();

    /** @brief change state depending on the module coupled on */
    virtual void updatePowerConsumption(bool on);

private:
    W _consumption;       // power consumption in state on
    W _offConsumption;    // power consumption in state off
    ConsumerStateHandler *_followedModule; // prt to the module, that this module follows
};

}  // namespace estnet

#endif /* ENERGYMODEL_CONSUMER_CONSUMERTYPES_COUPLEDCONSUMER_H_ */
