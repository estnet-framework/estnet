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

#ifndef ENERGYMODEL_CONSUMER_CONSUMERTYPES_PARAMRANGECONSUMER_H_
#define ENERGYMODEL_CONSUMER_CONSUMERTYPES_PARAMRANGECONSUMER_H_

#include <omnetpp.h>
#include <inet/power/contract/IEpEnergyConsumer.h>
#include <inet/common/Units.h>
#include <inet/common/geometry/common/Coord.h>

#include "estnet/power/base/ConsumerStateHandler.h"
#include "estnet/node/satellite/Satellite.h"
#include "estnet/environment/contract/IEarthModel.h"
#include "estnet/environment/earthmodel/EarthModelFactory.h"
#include "estnet/attitudecontroller/AttitudeController.h"
#include "estnet/attitudecontroller/AttitudeTarget.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::s> s;

/*
 *  Consumer class that consumes power if satellite is over target
 *  This power stays constant.
 *  The model requires the longitude and latitude of the target
 *  point. Additionally the minimal elevation is given.
 *  When flying over the target, this consumer is triggering the
 *  attitude controller.
 */
class ESTNET_API TargetTrackingConsumer: public ConsumerStateHandler {
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
    virtual ~TargetTrackingConsumer();

private:
    W _consumption;             ///< power consumption in state on
    W _offConsumption;           ///< power consumption in state off
    cMessage *_changeStatus;    ///< update message
    IEarthModel *_earthModel;   ///< earth model used for long/lat calculations
    inet::deg _latitude;        ///< latitude of observed point
    inet::deg _longitude;       ///< longitude of observed point
    inet::deg _minElevation;    ///< min. elevation to have to be able to observe the point
    AttitudeTarget _target;     ///< target for attitude pointing when passing the target area
    AttitudeTarget _lastTarget; ///< target for attitude pointing when leaving the target area
    /* @brief sends current target to attitude controller */
    void updateTracking();
};

}  // namespace estnet

#endif /* ENERGYMODEL_CONSUMER_CONSUMERTYPES_PARAMRANGECONSUMER_H_ */
