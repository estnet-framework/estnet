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

#ifndef SATMOBILITY_PROPAGATORS_POSITION_IPOSITIONPROPAGATOR_H_
#define SATMOBILITY_PROPAGATORS_POSITION_IPOSITIONPROPAGATOR_H_

#include <memory>

#include "IPropagatorBase.h"
#include "estnet/mobility/satellite/propagator/position/state/PropStatePosition.h"
#include "estnet/common/StlUtils.h"

namespace estnet {

/**
 * General interface for all position propagators.
 */

class ESTNET_API IPositionPropagator {
public:
    /**
     * Updates the position propagator state using ECI coordinates.
     * @param newPosition new position propagator state in eci coordinates
     * @param positionTimeStamp associated time stamp to the eci position
     */
    virtual void setStateFromECI(cEci const &newPosition,
            const cJulian &positionTimeStamp) = 0;

    /**
     * Computes the position at time targetTime and writes it into newECI as ECI
     * coordinates.
     * @param targetTime point in time to compute the position for
     * @param newECI computed position at time targetTime in ECI coordinates
     */
    virtual void getECIAtTime(cJulian const &targetTime, cEci &newECI) = 0;

    /**
     * Returns the orbital period of the current orbit.
     * @return orbital period in seconds
     */
    virtual double getOrbitalPeriod() const = 0;

    /**
     * Returns the current distance to the center of earth.
     * @return orbital radius in kilometers
     */
    virtual double getOrbitalRadius(cJulian const &targetTime) const = 0;
};

template<typename tPositionState>
class ESTNET_API PositionPropagator: public PropagatorBase<tPositionState>,
        public IPositionPropagator {
    // check if the state of this position propagator is a valid position state
    static_assert(std::is_base_of<PropStatePosition, tPositionState>::value,
            "tPositionState must extend PropStatePosition!");

public:
    using typename PropagatorBase<tPositionState>::state_type;

    /**
     * Updates the position propagator state using ECI coordinates.
     * @param newPosition new position propagator state in eci coordinates
     * @param positionTimeStamp associated time stamp to the eci position
     */
    virtual void setStateFromECI(cEci const &newPosition,
            const cJulian &positionTimeStamp) override
            {
        state_type tmpState;
        (static_cast<PropStatePosition*>(&tmpState))->fromECI(newPosition);
        (static_cast<PropStatePosition*>(&tmpState))->setTimeStamp(
                positionTimeStamp);
        tPropState_Ptr tmpStatePtr(&tmpState);
        PropagatorBase<tPositionState>::setState(tmpStatePtr);
    }

    /**
     * Computes the position at time targetTime and writes it into newECI as ECI
     * coordinates.
     * @param targetTime point in time to compute the position for
     * @param newECI computed position at time targetTime in ECI coordinates
     */
    virtual void getECIAtTime(cJulian const &targetTime, cEci &newECI) override
    {
        //check cached eci values
        std::map<cJulian, cEci>::const_iterator eciIt =
                this->_computedEcis.find(targetTime);
        if (eciIt != this->_computedEcis.end()) {
            newECI = eciIt->second;
            return;
        }
        // Calculate new state and save it to cache
        tPropState_Ptr tmpStatePtr;
        PropagatorBase<tPositionState>::getState(targetTime, tmpStatePtr);
        tPropStatePosition_Ptr tmpPosPtr = std::static_pointer_cast<
                PropStatePosition, PropState>(tmpStatePtr);
        tmpPosPtr->asECI(newECI);
        this->_computedEcis.emplace(targetTime, newECI);
    }

private:
    std::map<cJulian, cEci> _computedEcis;
};

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_POSITION_IPOSITIONPROPAGATOR_H_ */
