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

#ifndef ENERGYMODEL_CONSUMER_CONSTANTCONSUMER_H_
#define ENERGYMODEL_CONSUMER_CONSTANTCONSUMER_H_

#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/power/contract/IEpEnergySource.h>
#include <inet/common/Units.h>

#include "estnet/common/ESTNETDefs.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::W> W;

/**
 *  Consumer class that is always consuming power.
 *  This power stays constant.
 */
class ESTNET_API ConstantConsumer: public cSimpleModule,
        public inet::power::IEpEnergyConsumer {
private:
    /* stores current consumption value */
    W _consumption;

    /* prt to energy source module */
    inet::power::IEpEnergySource *_energySource = nullptr;

protected:
    /** @brief returns number of initialization stages
     *  @return int: number of initialization stages */
    virtual int numInitStages() const;

    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

public:
    /** @brief returns module, which is the energy source for this consumer
     *  @return IEnergySource: battery that is used by the consumer*/
    virtual inet::power::IEnergySource* getEnergySource() const override;

    /** @brief returns power consumed at the moment
     *  @return W: current power consumption in Watt */
    virtual W getPowerConsumption() const override;
};

}  // namespace estnet

#endif /* ENERGYMODEL_CONSUMER_CONSTANTCONSUMER_H_ */
