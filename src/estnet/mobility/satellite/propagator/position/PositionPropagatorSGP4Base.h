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

#ifndef __ESTNET_POSITIONPROPAGATORSGP4BASE_H_
#define __ESTNET_POSITIONPROPAGATORSGP4BASE_H_

#include <omnetpp.h>

#include "estnet/mobility/satellite/contract/IPositionPropagator.h"
#include "estnet/mobility/satellite/common/cEci.h"
#include "state/PropStateECI.h"
#include "estnet/mobility/satellite/common/sgp4/sgp4io.h"

namespace estnet {

/**
 * Implements a SGP4 position propagator.
 * Abstract base class with a child providing
 * the function +getTleLines+.
 */
class ESTNET_API PositionPropagatorSGP4Base: public omnetpp::cSimpleModule,
        public PositionPropagator<PropStateECI> {
private:
    ///< returns position[m] and velocity [m/s] at given time
    cEci GetState(cJulian const &targetTime) const;

    std::string _firstLine;
    std::string _secondLine;
    elsetrec _satrec;

protected:
    /**
     * must be overriden by children to provide the two TLE lines
     */
    virtual void getTleLines(std::string &firstLine,
            std::string &secondLine) = 0;

    /**
     * Initializes all the orbit-parameters and the 3D anmimation.
     * Schedules the first initial movement
     * @param stage of the initialization (2 steps needed)
     */
    virtual void initialize() override;

    /**
     * from cModule
     * handler for incomming messages. in this case as timer for the next position
     * update
     * @param msg incomming message
     */
    virtual void handleMessage(omnetpp::cMessage *msg) override;

    /**
     * Propagates the satellite state starting from the current state until the
     * targetTime point in time.
     * @param targetTime point in time for which the state should be propagated
     * @param newState new propagated state
     * Note: Must be implemented by the respective propagators
     */
    virtual void propagateState(cJulian const &targetTime, state_type &newState)
            override;

    /**
     * Returns whether or not an state propagation/update is necessary for the
     * state propagation targetTime, e.g. if the
     * current state is still within accuracy bounds for the propagation target
     * time.
     * @param targetTime target time for which a state update necessity should be
     * checked.
     * @return true, if an state propagation/update is necessary for the
     * targetTime point in time, false otherwise
     * Note: Must be implemented by the respective propagators
     */
    virtual bool needsStateUpdate(cJulian const &targetTime) override;

    /**
     * Returns the orbital period of the current orbit.
     * Implementation for the IPositionPropagator interface
     * @return orbital period in seconds
     */
    virtual double getOrbitalPeriod() const override;

    /**
     * Returns the current distance to the center of earth.
     * @return orbital radius in kilometers
     */
    virtual double getOrbitalRadius(cJulian const &targetTime) const override;
};

}  // namespace estnet

#endif /* __ESTNET_POSITIONPROPAGATORSGP4BASE_H_ */
