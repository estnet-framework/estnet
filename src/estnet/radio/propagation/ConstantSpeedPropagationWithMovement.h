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

/**
 * Implements a propagation that allows to take into account movements of the nodes
 * This is crucial for satellites, as fast velocities may result into
 * weird behavior otherwise
 */
class ESTNET_API ConstantSpeedPropagationWithMovement: public ConstantSpeedPropagation {
protected:
    bool ignoreMovementDuringTransmission;
    bool ignoreMovementDuringPropagation;
    bool ignoreMovementDuringReception;

protected:
    /**
     * Module initialization, that read decision in NED parameter, whether to use
     * or ignore movements
     * @param stage: current stage of initialization
     */
    virtual void initialize(int stage) override;

    /**
     * Computes the position of the node at arrival of the transmitted signal
     * @param startTime: start time of transmission
     * @param startPosition: start position of transmitting node
     * @param mobility: mobility of receiving node
     * @return Coord: position of receiving node at arrival
     *
     */
    virtual const Coord computeArrivalPosition(const simtime_t startTime,
            const Coord startPosition, IMobility *mobility) const override;

public:
    /**
     * Computes a arrival of a transmission at a receiving node
     * @param transmission: transmitted signal/packet
     * @param mobility: mobility of receiving node
     * @return IArrival: arrival of transmitted signal
     */
    virtual const IArrival* computeArrival(const ITransmission *transmission,
            IMobility *mobility) const override;
};

}  // namespace estnet

#endif

