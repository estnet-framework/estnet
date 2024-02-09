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

#include "ConstantSpeedPropagationWithMovement.h"

#include <inet/physicallayer/common/packetlevel/Arrival.h>
#include <inet/physicallayer/common/packetlevel/Radio.h>

#include "estnet/mobility/contract/IExtendedMobility.h"

namespace estnet {

Define_Module(ConstantSpeedPropagationWithMovement);

void ConstantSpeedPropagationWithMovement::initialize(int stage) {
    PropagationBase::initialize(stage);
    if (stage == INITSTAGE_LOCAL) {
        ignoreMovementDuringTransmission = par(
                "ignoreMovementDuringTransmission");
        ignoreMovementDuringPropagation = par(
                "ignoreMovementDuringPropagation");
        ignoreMovementDuringReception = par("ignoreMovementDuringReception");
    }
}

const Coord ConstantSpeedPropagationWithMovement::computeArrivalPosition(
        const simtime_t time, const Coord position, IMobility *mobility) const {
    auto extMobility = check_and_cast<IExtendedMobility*>(mobility);
    return extMobility->getPositionAtTime(time.dbl());
}

const IArrival* ConstantSpeedPropagationWithMovement::computeArrival(
        const ITransmission *transmission, IMobility *mobility) const {
    arrivalComputationCount++;
    const simtime_t startTime = transmission->getStartTime();
    const simtime_t endTime = transmission->getEndTime();
    const Coord startPosition = transmission->getStartPosition();
    auto transmitter = check_and_cast<const Radio*>(
            transmission->getTransmitter());
    const Coord endPosition =
            ignoreMovementDuringTransmission ?
                    transmission->getEndPosition() :
                    computeArrivalPosition(endTime, startPosition,
                            transmitter->getAntenna()->getMobility());
    const Coord startArrivalPosition =
            ignoreMovementDuringPropagation ?
                    mobility->getCurrentPosition() :
                    computeArrivalPosition(startTime, startPosition, mobility);
    const simtime_t startPropagationTime = startPosition.distance(
            startArrivalPosition) / propagationSpeed.get();
    const simtime_t startArrivalTime = startTime + startPropagationTime;
    const Quaternion startArrivalOrientation =
            mobility->getCurrentAngularPosition();
    if (ignoreMovementDuringReception) {
        const Coord endArrivalPosition = startArrivalPosition;
        const simtime_t endPropagationTime = startPropagationTime;
        const simtime_t endArrivalTime = endTime + startPropagationTime;
        const simtime_t preambleDuration = transmission->getPreambleDuration();
        const simtime_t headerDuration = transmission->getHeaderDuration();
        const simtime_t dataDuration = transmission->getDataDuration();
        const Quaternion endArrivalOrientation =
                mobility->getCurrentAngularPosition();
        return new Arrival(startPropagationTime, endPropagationTime,
                startArrivalTime, endArrivalTime, preambleDuration,
                headerDuration, dataDuration, startArrivalPosition,
                endArrivalPosition, startArrivalOrientation,
                endArrivalOrientation);
    } else {
        const Coord endArrivalPosition = computeArrivalPosition(endTime,
                endPosition, mobility);
        const simtime_t endPropagationTime = endPosition.distance(
                endArrivalPosition) / propagationSpeed.get();
        const simtime_t endArrivalTime = endTime + endPropagationTime;
        const simtime_t preambleDuration = transmission->getPreambleDuration();
        const simtime_t headerDuration = transmission->getHeaderDuration();
        const simtime_t dataDuration = transmission->getDataDuration();
        const Quaternion endArrivalOrientation =
                mobility->getCurrentAngularPosition();
        return new Arrival(startPropagationTime, endPropagationTime,
                startArrivalTime, endArrivalTime, preambleDuration,
                headerDuration, dataDuration, startArrivalPosition,
                endArrivalPosition, startArrivalOrientation,
                endArrivalOrientation);
    }
}

}  // namespace estnet
