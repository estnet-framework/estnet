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

#ifndef SATMOBILITY_SATMOBILITY_H_
#define SATMOBILITY_SATMOBILITY_H_

#include <string>

#include <inet/mobility/contract/IMobility.h>

#include "estnet/mobility/contract/IExtendedMobility.h"
#include "estnet/mobility/satellite/contract/IAttitudePropagator.h"
#include "estnet/mobility/satellite/propagator/attitude/state/PropStateAttitude.h"
#include "estnet/mobility/satellite/contract/IPositionPropagator.h"
#include "estnet/mobility/satellite/propagator/position/state/PropStatePosition.h"
#include "estnet/mobility/satellite/common/cEci.h"

namespace estnet {

class ESTNET_API SatMobility: public IExtendedMobility {
public:
    SatMobility();
    virtual ~SatMobility() = default;
    /**
     * Public member functions used by other modules for direct attitude/position
     * access and interaction with this module.
     */

    /**
     * Returns the current position at the given simulation time (e.g. to create a
     * orbit-circle in OSG-Canvas)
     * @param time target simulation time in seconds
     * @return Position at given simulation time
     */
    virtual inet::Coord getPositionAtTime(double time);

    virtual inet::Coord getVelocityAtTime(double time);

    virtual inet::Quaternion getAngularPositionAtTime(double time);

    virtual inet::Quaternion getAngularVelocityAtTime(double time);

    /**
     * Returns the current position without emitting a signal.
     * Should only be called by subscribed modules in their
     * receive method.
     */
    virtual inet::Coord getCurrentPositionWithoutSignal();

    /**
     * Returns the orbital period of the current orbit
     *@return orbital period in seconds
     */
    virtual int getOrbitalPeriod() const;

    /**
     * Returns the current distance to the center of earth.
     * @return orbital radius in kilometers
     */
    virtual double getOrbitalRadius() const;

    /**
     * This function can be called from an external source/module to update the
     * time stamped position state.
     * @param newPosition updated time stamped position state
     */
    void externalPositionUpdate(tPropStatePosition_Ptr const &newPosition);
    /**
     * This function can be called from an external source/module to update the
     * position and the associated time.
     * @param newPosition position update
     * @param positionTimeStamp time stamp associated with the position update
     */
    void externalPositionUpdateECI(cEci const &newPosition,
            cJulian const &positionTimeStamp);
    /**
     * This function can be called from an external source/module to update the
     * time stamped attitude state.
     * @param newAttitude updated time stamped attitude state
     */
    void externalAttitudeUpdate(tPropStateAttitude_Ptr const &newAttitude);

    /**
     * This function can be called from an external source/module to update the
     * attitude and the associated time.
     * @param newAttitude attitude update
     * @param newAngularVel new angular velocity state update in euler angles
     * @param newAngularAcc new angular acceleration state update in euler angles
     * @param attitudeTimeStamp time stamp associated with the attitude update
     */
    void externalAttitudeUpdateEuler(cEulerAngles const &newAttitude,
    cEulerAngles const &newAngularVel,
    cEulerAngles const &newAngularAcc, cJulian const &attitudeTimeStamp);

    /**
     * This function can be called from an external source/module to update the
     * attitude and the associated time.
     * @param newAttitude attitude update
     * @param newAngularVel new angular velocity state update in quaternions
     * @param newAngularAcc new angular acceleration state update in quaternions
     * @param attitudeTimeStamp time stamp associated with the attitude update
     */
    void externalAttitudeUpdateQuaternion(cQuaternion const &newAttitude,
    cQuaternion const &newAngularVel,
    cQuaternion const &newAngularAcc, cJulian const &attitudeTimeStamp);

    /**
     * Virtual functions from IMobility superclass
     */

    /** @brief Returns the maximum possible speed at any future time. */
    virtual double getMaxSpeed() const override;

    /** @brief Returns the current position at the current simulation time. */
    virtual inet::Coord getCurrentPosition() override;

    /** @brief Returns the current velocity at the current simulation time. */
    virtual inet::Coord getCurrentVelocity() override;

    /** @brief Returns the current acceleration at the current simulation time. */
    virtual inet::Coord getCurrentAcceleration() override;

    /** @brief Returns the current angular position at the current simulation
     * time. */
    virtual inet::Quaternion getCurrentAngularPosition() override;

    /** @brief Returns the current angular velocity at the current simulation time.
     */
    virtual inet::Quaternion getCurrentAngularVelocity() override;

    /** @brief Returns the current angular acceleration at the current simulation time.
     */
    virtual inet::Quaternion getCurrentAngularAcceleration() override;

    /** @brief Returns the constraint area maximum
     */
    virtual inet::Coord getConstraintAreaMax() const override;
    /** @brief Returns the constraint are minimum
     */
    virtual inet::Coord getConstraintAreaMin() const override;

protected:
    virtual void initialize(int stage) override;

private:
    /*
    *   @brief Iterates through one orbit and extract maximum velocity
    *   @return double: maximum velocity of satellite
    */
    double estimateMaximumVelocity();

    IAttitudePropagator *_iAttitudePropagator; ///< pointer to the attitude propagator
    IPositionPropagator *_iPositionPropagator; ///< pointer to the position propagator

    std::string extUpdtSignalNamePrefix; ///< name prefix for the signal used to
                                         /// perform an external position/attitude
    /// update in this satmobility module
    inet::EulerAngles _coordinateFrame;

    std::map<int64_t, cQuaternion> _cachedAngularPositions;

    /// signals for statistics
    static omnetpp::simsignal_t positionUpdateX;
    static omnetpp::simsignal_t positionUpdateY;
    static omnetpp::simsignal_t positionUpdateZ;
    static omnetpp::simsignal_t velocityUpdateX;
    static omnetpp::simsignal_t velocityUpdateY;
    static omnetpp::simsignal_t velocityUpdateZ;

    double _maximumVelocity; ///< stores the maximum velocity that is calculated in the beginnings
};

}  // namespace estnet

#endif /* SATMOBILITY_SATMOBILITY_H_ */
