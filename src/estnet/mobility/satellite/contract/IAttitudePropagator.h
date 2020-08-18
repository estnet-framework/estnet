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

#ifndef SATMOBILITY_PROPAGATORS_ATTITUDE_IATTITUDEPROPAGATOR_H_
#define SATMOBILITY_PROPAGATORS_ATTITUDE_IATTITUDEPROPAGATOR_H_

#include "estnet/mobility/satellite/contract/IPropagatorBase.h"
#include "estnet/mobility/satellite/propagator/attitude/state/PropStateAttitude.h"
#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"

namespace estnet {

/**
 * General interface for all attitude propagators.
 */

// typedef PropStateQuaternion tAttitudeState; // Set the default state for
// attitude propagators to quaternions
class ESTNET_API IAttitudePropagator {
public:
    virtual ~IAttitudePropagator() {
    }
    ;

    /**
     * Updates the attitude propagator state using Euler angles.
     * @param newAttitude new attitude propagator state in euler angles
     * @param newAngularVel new angular velocity state in euler angles
     * @param newAngularAcc new angular acceleration state in euler angles
     * @param attitudeTimeStamp associated time stamp to the euler angle attitude
     */
    virtual void setStateFromEuler(cEulerAngles const &newAttitude,
    cEulerAngles const &newAngularVel,
    cEulerAngles const &newAngularAcc, const cJulian &attitudeTimeStamp) = 0;

    /**
     * Updates the attitude propagator state using Quaternions.
     * @param newAttitude new attitude propagator state in quaternions
     * @param newAngularVel new angular velocity state in quaternions
     * @param newAngularAcc new angular acceleration state in quaternions
     * @param attitudeTimeStamp associated time stamp to the quaternion attitude
     */
    virtual void setStateFromQuaternion(cQuaternion const &newAttitude,
    cQuaternion const &newAngularVel,
    cQuaternion const &newAngularAcc, const cJulian &attitudeTimeStamp) = 0;

    /**
     * Computes the attitude at time targetTime and writes it into newEuler as
     * euler angles.
     * @param targetTime point in time to compute the attitude for
     * @param newEuler computed attitude at time targetTime in euler angles
     */
    virtual void getEulerAttitudeAtTime(cJulian const &targetTime,
    cEulerAngles &newEuler) = 0;

    /**
     * Computes the attitude at time targetTime and writes it into newQuaternion as
     * quaternions.
     * @param targetTime point in time to compute the attitude for
     * @param newQuaternion computed attitude at time targetTime in quaternions
     */
    virtual void getQuaternionAttitudeAtTime(cJulian const &targetTime,
    cQuaternion &newQuaternion) = 0;

    /**
     * Computes the angular velocity at time targetTime and writes it into
     * newEuler as euler angles.
     * @param targetTime point in time to compute the angular velocity for
     * @param newEuler computed angular velocity at time targetTime in euler
     * angles
     */
    virtual void getEulerAngularVelAtTime(cJulian const &targetTime,
    cEulerAngles &newEuler) = 0;

    /**
     * Computes the angular velocity at time targetTime and writes it into
     * newQuaternion as quaternions.
     * @param targetTime point in time to compute the angular velocity for
     * @param newQuaternion computed angular velocity at time targetTime in quaternions
     */
    virtual void getQuaternionAngularVelAtTime(cJulian const &targetTime,
    cQuaternion &newQuaternion) = 0;

    /**
     * Computes the angular acceleration at time targetTime and writes it into
     * newEuler as euler angles.
     * @param targetTime point in time to compute the angular acceleration for
     * @param newEuler computed angular acceleration at time targetTime in euler
     * angles
     */
    virtual void getEulerAngularAccAtTime(cJulian const &targetTime,
    cEulerAngles &newEuler) = 0;

    /**
     * Computes the angular acceleration at time targetTime and writes it into
     * newQuaternion as quaternions.
     * @param targetTime point in time to compute the angular acceleration for
     * @param newQuaternion computed angular acceleration at time targetTime in quaternions
     */
    virtual void getQuaternionAngularAccAtTime(cJulian const &targetTime,
    cQuaternion &newQuaternion) = 0;
};

template<typename tAttitudeState>
class ESTNET_API AttitudePropagator: public PropagatorBase<tAttitudeState>,
        public IAttitudePropagator {
    // check if the state of this attitude propagator is a valid attitude state
    static_assert(std::is_base_of<PropStateAttitude, tAttitudeState>::value,
            "tAttitudeState must extend PropStateAttitude!");

public:
    // virtual static double projectAttitudeToHeading(tAttitudeState &acAttitude)
    // = 0;

    using typename PropagatorBase<tAttitudeState>::state_type;
    using typename PropagatorBase<tAttitudeState>::state_ptr;

    /**
     * Constructor of the generic AttitudePropagator
     */
    AttitudePropagator() :
            PropagatorBase<tAttitudeState>(), IAttitudePropagator() {
    }

    /**
     * Updates the attitude propagator state using Euler angles.
     * @param newAttitude new attitude propagator state in euler angles
     * @param newAngularVel new angular velocity state in euler angles
     * @param newAngularAcc new angular acceleration state in euler angles
     * @param attitudeTimeStamp associated time stamp to the euler angle attitude
     */
    virtual void setStateFromEuler(cEulerAngles const &newAttitude,
    cEulerAngles const &newAngularVel,
    cEulerAngles const &newAngularAcc, const cJulian &attitudeTimeStamp)
            override
            {
        state_ptr tmpStatePtr = std::make_shared<state_type>();
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->attitudeFromEulerAngles(
                newAttitude);
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->angularVelFromEulerAngles(
                newAngularVel);
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->angularAccFromEulerAngles(
                newAngularAcc);
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->setTimeStamp(
                attitudeTimeStamp);
        PropagatorBase<tAttitudeState>::setState(
                std::dynamic_pointer_cast<PropState, state_type>(tmpStatePtr));
    }

    /**
     * Updates the attitude propagator state using Quaternions.
     * @param newAttitude new attitude propagator state in quaternions
     * @param newAngularVel new angular velocity state in quaternions
     * @param newAngularAcc new angular acceleration state in quaternions
     * @param attitudeTimeStamp associated time stamp to the quaternion attitude
     */
    virtual void setStateFromQuaternion(cQuaternion const &newAttitude,
    cQuaternion const &newAngularVel,
    cQuaternion const &newAngularAcc, const cJulian &attitudeTimeStamp) override
    {
        state_ptr tmpStatePtr = std::make_shared<state_type>();
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->attitudeFromQuaternion(
                newAttitude);
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->angularVelFromQuaternion(
                newAngularVel);
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->angularAccFromQuaternion(
                newAngularAcc);
        std::dynamic_pointer_cast<PropStateAttitude, state_type>(tmpStatePtr)->setTimeStamp(
                attitudeTimeStamp);
        PropagatorBase<tAttitudeState>::setState(
                std::dynamic_pointer_cast<PropState, state_type>(tmpStatePtr));
    }

    /**
     * Computes the attitude at time targetTime and writes it into newEuler as
     * euler angles.
     * @param targetTime point in time to compute the attitude for
     * @param newEuler computed attitude at time targetTime in euler angles
     */
    virtual void getEulerAttitudeAtTime(cJulian const &targetTime,
    cEulerAngles &newEuler) override
    {
        // TODO: check if we really need to compute new euler angle values, or if we
        // use the last one in the cache...
        tPropState_Ptr tmpPropStatePtr;
        PropagatorBase<tAttitudeState>::getState(targetTime, tmpPropStatePtr);
        tPropStateAttitude_Ptr tmpAttPtr = std::static_pointer_cast<
                PropStateAttitude, PropState>(tmpPropStatePtr);
        tmpAttPtr->attitudeAsEulerAngles(newEuler);
    }

    /**
     * Computes the attitude at time targetTime and writes it into newQuaternion as
     * quaternion.
     * @param targetTime point in time to compute the attitude for
     * @param newQuaternion computed attitude at time targetTime in quaternions
     */
    virtual void getQuaternionAttitudeAtTime(cJulian const &targetTime,
    cQuaternion &newQuaternion) override
    {
        // TODO: check if we really need to compute new quaternion values, or if we
        // use the last one in the cache...
        tPropState_Ptr tmpPropStatePtr;
        PropagatorBase<tAttitudeState>::getState(targetTime, tmpPropStatePtr);
        tPropStateAttitude_Ptr tmpAttPtr = std::static_pointer_cast<
                PropStateAttitude, PropState>(tmpPropStatePtr);
        tmpAttPtr->attitudeAsQuaternion(newQuaternion);
    }

    /**
     * Computes the angular velocity at time targetTime and writes it into
     * newEuler as euler angles.
     * @param targetTime point in time to compute the angular velocity for
     * @param newEuler computed angular velocity at time targetTime in euler
     * angles
     */
    virtual void getEulerAngularVelAtTime(cJulian const &targetTime,
    cEulerAngles &newEuler) override
    {
        // TODO: check if we really need to compute new euler angle values, or if we
        // use the last one in the cache...
        tPropState_Ptr tmpStatePtr;
        PropagatorBase<tAttitudeState>::getState(targetTime, tmpStatePtr);
        tPropStateAttitude_Ptr tmpAttPtr = std::static_pointer_cast<
                PropStateAttitude, PropState>(tmpStatePtr);
        tmpAttPtr->angularVelAsEulerAngles(newEuler);
    }

    /**
     * Computes the angular velocity at time targetTime and writes it into
     * newQuaternion as quaternion.
     * @param targetTime point in time to compute the angular velocity for
     * @param newQuaternion computed angular velocity at time targetTime in quaternions
     */
    virtual void getQuaternionAngularVelAtTime(cJulian const &targetTime,
    cQuaternion &newQuaternion) override
    {
        // TODO: check if we really need to compute new quaternion values, or if we
        // use the last one in the cache...
        tPropState_Ptr tmpStatePtr;
        PropagatorBase<tAttitudeState>::getState(targetTime, tmpStatePtr);
        tPropStateAttitude_Ptr tmpAttPtr = std::static_pointer_cast<
                PropStateAttitude, PropState>(tmpStatePtr);
        tmpAttPtr->angularVelAsQuaternion(newQuaternion);
    }

    /**
     * Computes the angular acceleration at time targetTime and writes it into
     * newEuler as euler angles.
     * @param targetTime point in time to compute the angular acceleration for
     * @param newEuler computed angular acceleration at time targetTime in euler
     * angles
     */
    virtual void getEulerAngularAccAtTime(cJulian const &targetTime,
    cEulerAngles &newEuler) override
    {
        // TODO: check if we really need to compute new euler angle values, or if we
        // use the last one in the cache...
        tPropState_Ptr tmpStatePtr;
        PropagatorBase<tAttitudeState>::getState(targetTime, tmpStatePtr);
        tPropStateAttitude_Ptr tmpAttPtr = std::static_pointer_cast<
                PropStateAttitude, PropState>(tmpStatePtr);
        tmpAttPtr->angularAccAsEulerAngles(newEuler);
    }

    /**
     * Computes the angular acceleration at time targetTime and writes it into
     * newQuaternion as quaternion.
     * @param targetTime point in time to compute the angular acceleration for
     * @param newQuaternion computed angular acceleration at time targetTime in quaternions
     */
    virtual void getQuaternionAngularAccAtTime(cJulian const &targetTime,
    cQuaternion &newQuaternion) override
    {
        // TODO: check if we really need to compute new quaternion values, or if we
        // use the last one in the cache...
        tPropState_Ptr tmpStatePtr;
        PropagatorBase<tAttitudeState>::getState(targetTime, tmpStatePtr);
        tPropStateAttitude_Ptr tmpAttPtr = std::static_pointer_cast<
                PropStateAttitude, PropState>(tmpStatePtr);
        tmpAttPtr->angularAccAsQuaternion(newQuaternion);
    }
};

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_ATTITUDE_IATTITUDEPROPAGATOR_H_ */
