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

#ifndef __ESTNET_POSITIONPROPAGATORLINEAR_H_
#define __ESTNET_POSITIONPROPAGATORLINEAR_H_

#include "estnet/mobility/satellite/contract/IPositionPropagator.h"
#include "state/PropStateECI.h"

namespace estnet {

/**
 * This is a simple linear position propagator as an example position
 * propagator.
 * It uses ECI coordinates as internal state...
 */
class ESTNET_API PositionPropagatorLinear: public omnetpp::cSimpleModule,
        PositionPropagator<PropStateECI> {
protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;

protected:
    double _minStateAccuracy;

    /**
     *
     *
     * Propagator methods inherited from IPropagatorBase.
     * Here the actual propagation takes place.
     *
     *
     */

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

#endif
