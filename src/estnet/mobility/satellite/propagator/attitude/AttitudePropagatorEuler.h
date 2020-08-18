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

#ifndef __ESTNET_ATTITUDEPROPAGATOR_EULER_H_
#define __ESTNET_ATTITUDEPROPAGATOR_EULER_H_

#include "estnet/mobility/satellite/contract/IAttitudePropagator.h"
#include "estnet/mobility/satellite/propagator/attitude/state/PropStateEulerAngles.h"

namespace estnet {

class ESTNET_API AttitudePropagatorEuler: public omnetpp::cSimpleModule,
        public AttitudePropagator<PropStateEulerAngles> {
public:
    AttitudePropagatorEuler();

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

protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
};

}  // namespace estnet

#endif
