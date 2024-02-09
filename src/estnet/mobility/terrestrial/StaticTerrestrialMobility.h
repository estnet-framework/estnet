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

#ifndef SRC_SATMOBILITY_STATICTERRESTRIALMOBILITY_H_
#define SRC_SATMOBILITY_STATICTERRESTRIALMOBILITY_H_

#include <omnetpp.h>

#include "estnet/mobility/contract/IExtendedMobility.h"
#include "estnet/environment/contract/IEarthModel.h"
#include "estnet/common/time/GlobalJulianDate.h"

namespace estnet {

/**
 * The StaticTerrestrialMobility module provides position
 * calculation (earth roatation) for surface-fixed communication
 * devices (e.g. GroundStations) and 3D visualization in OSG
 */
class ESTNET_API StaticTerrestrialMobility: public IExtendedMobility {
public:
    /** @brief Sets the attitude of the node to the given value */
    virtual void setCurrentAngularPosition(
            const inet::Quaternion &newOrientation);

    /**
     * Returns the current position at the given simulation time (e.g. to create a
     * orbit-circle in OSG-Canvas)
     * @param time target simulation time in seconds
     * @return Position at given simulation time
     */
    virtual inet::Coord getPositionAtTime(double time);

    virtual inet::Coord getVelocityAtTime(double time);

    /**
     *  Computes the current azimuth and elevation angle
     *  @param reference to the computed azimuth angle [rad]
     *  @param reference to the computed elevation angle [rad]
     */
    virtual void getCurrentOrientationAngles(double &azimuth, double &elevation);

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
    virtual int numInitStages() {
        return inet::NUM_INIT_STAGES;
    }
    /** @brief destructor */
    virtual ~StaticTerrestrialMobility();
    /** @brief called at initialization */
    virtual void initialize(int stage) override;

private:
    inetu::deg lon;
    inetu::deg lat;
    inetu::m alt;
    bool enableKinematics;
    double maxAzRate, maxElRate;
    cJulian _lastUpdateTime;

    static omnetpp::simsignal_t az;
    static omnetpp::simsignal_t el;
    static omnetpp::simsignal_t azEr;
    static omnetpp::simsignal_t elEr;

    static omnetpp::simsignal_t positionUpdateX;
    static omnetpp::simsignal_t positionUpdateY;
    static omnetpp::simsignal_t positionUpdateZ;

    static omnetpp::simsignal_t velocityUpdateX;
    static omnetpp::simsignal_t velocityUpdateY;
    static omnetpp::simsignal_t velocityUpdateZ;

    inet::Quaternion _orientation;
    IEarthModel *_earthModel;
};

}  // namespace estnet
#endif /* SRC_SATMOBILITY_STATICTERRESTRIALMOBILITY_H_ */
