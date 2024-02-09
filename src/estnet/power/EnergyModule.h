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

#ifndef ENERGYMODEL_ENERGYMODULE_H_
#define ENERGYMODEL_ENERGYMODULE_H_

#include <omnetpp.h>

#include <inet/common/geometry/common/Coord.h>

#include "estnet/common/ESTNETDefs.h"
#include "estnet/mobility/contract/IExtendedMobility.h"

namespace estnet {

/**
 * Describes power budget on satellite
 * takes care of all power consumer and producer
 * could be used for management like what happens in case of an empty battery
 * or make a decision on supplying consumers direct without loading battery
 * Additionally it is determining if the satellite is in the earth's shadow
 *
 */
class ESTNET_API EnergyModule: public omnetpp::cModule {
public:
    /** @brief checks whether satellite in in Earth's shadow
     *         by checking whether any power is generated
     *  @return bool: true if satellite is in eclipse */
    bool isInEclipse();

    /** gets the suns position in ECI frame
     *  which is used to calculate the suns illumination angle
     *  of the solar cells
     *  Idea for further improvement: model real coordinate of
     *  the sun dependent on the time
     *  @return inet::Coord: vector to sun in ECI frame */
    inet::Coord getSunPosition();

protected:
    /** @brief initialization of the module, called by omnet
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;

private:
    inet::Coord _sunPosition;   // suns coordinates in ECI, stays at this point
    IExtendedMobility *_mobility; // the satellites mobility
    bool _supplyConsumerDirectly;   // not used, intended to alter efficiencies
    int _checkInterval; // not used

};

}  // namespace estnet

#endif /* ENERGYMODEL_ENERGYMODULE_H_ */
