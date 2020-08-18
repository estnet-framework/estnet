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

#ifndef __ESTNET_EXTERNCONSUMER_H_
#define __ESTNET_EXTERNCONSUMER_H_

#include <omnetpp.h>
#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/power/contract/IEpEnergySource.h>
#include <inet/common/Units.h>

#include "estnet/siminterface/pubsub/Subscriber.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::W> W;

/**
 * TODO consumer sets consumption to what is send through the pub/sub sytem
 */
class ESTNET_API ExternConsumer: public cSimpleModule,
        public Subscriber,
        public inet::power::IEpEnergyConsumer {
private:
    W _consumption;
    inet::power::IEpEnergySource *_energySource = nullptr;
protected:
    /** @brief returns number of initialization stages */
    virtual int numInitStages() const override;

    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

    /** @brief handles msg from extern interface
     *  @param pubSubMsg: message from exterm interface*/
    virtual void receivedPubSubMessage(PubSubMsg *pubSubMsg);

public:
    /** @brief returns module, which is the energy source for this consumer
     *  @return IEnergySource: battery that is used by the consumer*/
    virtual inet::power::IEnergySource* getEnergySource() const override;

    /** @brief returns power consumed at the moment
     *  @return W: current power consumption in Watt */
    virtual W getPowerConsumption() const override;
};

}  // namespace estnet

#endif
