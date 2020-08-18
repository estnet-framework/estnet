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

#ifndef ESTNET_POWER_CONSUMER_TYPE_SCHEDULEDCONSUMER_H_
#define ESTNET_POWER_CONSUMER_TYPE_SCHEDULEDCONSUMER_H_

#include <map>
#include <fstream>
#include <iostream>
#include <string>

#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/common/Units.h>
#include "estnet/power/base/ConsumerStateHandler.h"
#include "estnet/node/satellite/Satellite.h"

namespace estnet {

struct ConsumptionProperties {
    double time;
    W consumption;
    bool on;
};

/**
 * Scheduled consumer is reading in a .csv file.
 * This file is containing all relevant consumptions
 * for each timestep
 * This is how such a file can look like:
 * "sep=,
 *  Time,Consumption_Sat1,Consumption_Sat2
 *  0,1,1
 *  10,0,0
 *  20,1,1
 *  30,1,1"
 */
class ESTNET_API ScheduledConsumer: public ConsumerStateHandler {
protected:
    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

    /** @brief scheduled self-messages receiver function
     *  @param  message: message that is received */
    virtual void handleMessage(cMessage *message) override;

    /** @brief change state and power consumption */
    virtual void updatePowerConsumption();

    /// iterator for iterating over schedule
    std::vector<ConsumptionProperties>::iterator _currentSchedule;

    /// schedule that is stored in a vector
    std::vector<ConsumptionProperties> _scheduledConsumption;

    /// message for scheduling next consumption change
    cMessage *_updateTimer;

    /// timestep used in csv
    double _timestep;

public:
    /** @brief cancel and delete scheduled messages */
    virtual ~ScheduledConsumer();

};

} /* namespace estnet */

#endif /* ESTNET_POWER_CONSUMER_TYPE_SCHEDULEDCONSUMER_H_ */
