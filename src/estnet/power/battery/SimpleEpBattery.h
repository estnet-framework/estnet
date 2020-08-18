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

#ifndef __ESTNET_SIMPLEEPBATTERY_H_
#define __ESTNET_SIMPLEEPBATTERY_H_

#include <inet/power/storage/SimpleEpEnergyStorage.h>
#include <inet/common/Units.h>

#include "estnet/common/ESTNETDefs.h"

using namespace omnetpp;

namespace estnet {

typedef inet::units::value<double, inet::units::units::J> J;

/**
 * Simple Battery class describes energy storage
 * allows to decrease residual capacity by an amount of energy
 */
class ESTNET_API SimpleEpBattery: public inet::power::SimpleEpEnergyStorage {
public:
    /** @brief function for reducing the residual capacity of the battery by an amount of energy
     *  @param energy: energy portion that is removed in Joule*/
    virtual void removeEnergyPortion(J energy);

    /** @brief same function as in inet, but without simtime bug */
    virtual void scheduleTimer() override;
};

}  // namespace estnet

#endif
