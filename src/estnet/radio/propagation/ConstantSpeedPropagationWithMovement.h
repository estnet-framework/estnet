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

#ifndef __ESTNET_CONSTANTSPEEDPROPAGATIONWITHMOVEMENT_H_
#define __ESTNET_CONSTANTSPEEDPROPAGATIONWITHMOVEMENT_H_

#include <inet/physicallayer/propagation/ConstantSpeedPropagation.h>

#include "estnet/common/ESTNETDefs.h"

using namespace omnetpp;
using namespace inet;
using namespace physicallayer;

namespace estnet {

class ESTNET_API ConstantSpeedPropagationWithMovement: public ConstantSpeedPropagation {
protected:
    bool ignoreMovementDuringTransmission;
    bool ignoreMovementDuringPropagation;
    bool ignoreMovementDuringReception;

protected:
    virtual void initialize(int stage) override;
    virtual const Coord computeArrivalPosition(const simtime_t startTime,
            const Coord startPosition, IMobility *mobility) const override;

public:

    virtual const IArrival* computeArrival(const ITransmission *transmission,
            IMobility *mobility) const override;
};

}  // namespace estnet

#endif

