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

#ifndef ENERGYMODELINET_CONSUMER_CONSUMERTYPES_VARYINGDUTYCYCLECONSUMERINET_H_
#define ENERGYMODELINET_CONSUMER_CONSUMERTYPES_VARYINGDUTYCYCLECONSUMERINET_H_

#include <omnetpp.h>
#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/common/Units.h>

#include "estnet/power/base/ConsumerStateHandler.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::s> s;

/*
 *  Consumer class that has varying periods of consuming power.
 *  The power stays constant.
 *  The model is based on the mean time of activation, the
 *  mean time to reactivation and the standard deviation
 */
class ESTNET_API VaryingDutyCycleConsumer: public ConsumerStateHandler {
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
    virtual ~VaryingDutyCycleConsumer();

private:
    W _consumption;      // power consumption in state on
    W _offConsumption;   // power consumption in state off
    s _onTime;           // time the node is in state on
    s _offTime;          // time the node is in state off
    double _std;         // standard deviation of on/off times
    cMessage *_changeStatus; // update message
};

}  // namespace estnet

#endif /* ENERGYMODELINET_CONSUMER_CONSUMERTYPES_VARYINGDUTYCYCLECONSUMERINET_H_ */
