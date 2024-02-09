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

#ifndef SATMOBILITY_PROPAGATORS_IPROPAGATORBASE_H_
#define SATMOBILITY_PROPAGATORS_IPROPAGATORBASE_H_

#include <mutex>

#include "estnet/mobility/satellite/propagator/PropState.h"
#include "estnet/common/time/cJulian.h"

namespace estnet {

class ESTNET_API IPropagatorBase {
public:
    /**
     * Returns the state of the propagator for the targetTime point in time.
     * @param targetTime point in time for which the state should be propagated to
     * @param newState state of the propagator for the targetTime point in time
     */
    virtual void getState(cJulian const &targetTime,
            tPropState_Ptr &newState) = 0;

    /**
     * Sets the current propagator state at time targetTime from an external
     * source.
     * @param newState  new state for time targetTime
     */
    virtual void setState(tPropState_Ptr const &newState) = 0;

    /**
     * Initialization routine of the propagator. It is being called at the
     * beginning of the simulation by
     * the SatMobility module.
     * @param initialState initial state to which the internal state of the
     * propagator should be set to. Null of no initial value should be set.
     */
    virtual void initializePropagator(tPropState_Ptr initialState =
            tPropState_Ptr(nullptr)) = 0;
};

template<typename tState> class PropagatorBase: public IPropagatorBase {
public:
    typedef tState state_type;
    typedef std::shared_ptr<tState> state_ptr;

    PropagatorBase<tState>() :
            IPropagatorBase() {
    }

    /**
     * Allocates memory and returns a shared pointer to a new state instance.
     * @return shared pointer to a newly allocated state instance
     */
    virtual state_ptr allocNewState() {
        return state_ptr(new tState());
    }

    /**
     * Returns the state of the propagator for the targetTime point in time.
     * @param targetTime point in time for which the state should be propagated to
     * @param newState state of the propagator for the targetTime point in time
     */
    virtual void getState(cJulian const &targetTime, tPropState_Ptr &newState)
            override;

    /**
     * Sets the current propagator state at time targetTime from an external
     * source.
     * @param newState  new state for time targetTime
     */
    virtual void setState(tPropState_Ptr const &newState) override;

    /**
     * Initialization routine of the propagator. It is being called at the
     * beginning of the simulation by
     * the SatMobility module.
     * @param initialState initial state to which the internal state of the
     * propagator should be set to. Null of no initial value should be set.
     */
    virtual void initializePropagator(tPropState_Ptr initialState =
            tPropState_Ptr(nullptr)) override;

protected:
    cJulian _lastUpdateTime; // Last time the propagator state was updated (Julian time stamp)
    tState _currentState; // The current propagated state of the propagator for time _lastUpdateTime
    std::mutex _supdate_mutex; // State update lock (e.g. during propagation or external updates)
    double _minSAccuracy; // Minimal accuracy for each component of the state vector above which a new propagation is triggered

    /**
     * Propagates the satellite state starting from the current state until the
     * targetTime point in time.
     * @param targetTime point in time for which the state should be propagated
     * @param newState new propagated state
     * Note: Must be implemented by the respective propagators
     */
    virtual void propagateState(cJulian const &targetTime,
            state_type &newState) = 0;

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
    virtual bool needsStateUpdate(cJulian const &targetTime) = 0;
};

template<typename tState>
void PropagatorBase<tState>::getState(cJulian const &targetTime,
        tPropState_Ptr &newState) {
    std::unique_lock<std::mutex> _supdate_lock(_supdate_mutex);
    if (needsStateUpdate(targetTime)) {
        propagateState(targetTime, _currentState);
        _lastUpdateTime = targetTime;
    }

    auto tmpState = new PropagatorBase<tState>::state_type();
    *tmpState = _currentState;
    tPropState_Ptr nState = tPropState_Ptr(dynamic_cast<PropState*>(tmpState));
    newState = nState;
}

template<typename tState>
void PropagatorBase<tState>::setState(const tPropState_Ptr &newState) {
    std::unique_lock<std::mutex> _supdate_lock(_supdate_mutex);
    _currentState = *PropState::as<state_type>(newState.get());
    _lastUpdateTime = newState->getTimeStamp();
}

template<typename tState>
void PropagatorBase<tState>::initializePropagator(tPropState_Ptr initialState) {
    std::unique_lock<std::mutex> _supdate_lock(_supdate_mutex);
    // set minimal state accuracy to default value (0.0) -> perform a state update
    // every time
    _minSAccuracy = 0.0;
    if (initialState != nullptr) {
        _currentState = *PropState::as<state_type>(initialState.get());
    }
}

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_IPROPAGATORBASE_H_ */
