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

#ifndef __MOBILITY_OFFSET_MOBILITY_H__
#define __MOBILITY_OFFSET_MOBILITY_H__

#include <inet/common/Units.h>

#include "estnet/mobility/contract/IExtendedMobility.h"

namespace estnet {

/**
 * Mobility class for antennas
 * It combines mobility of the node it is contained in
 * with its own mobility.
 */
class ESTNET_API OffsetMobility: public IExtendedMobility {
protected:
    IExtendedMobility *_nodeMobility;
    inet::Coord _offset;
    inet::Quaternion _orientation;

public:
    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief Returns the maximum possible speed at any future time. */
    virtual double getMaxSpeed() const override;

    /** @brief Returns the current position at the current simulation time. */
    virtual inet::Coord getCurrentPosition() override;

    /** @brief Returns the current speed at the current simulation time. */
    virtual inet::Coord getCurrentVelocity() override;

    /** @brief Returs the current acceleration at the current simulation time. */
    virtual inet::Coord getCurrentAcceleration() override;

    /** @brief Returns the current angular position at the current simulation
     * time. */
    virtual inet::Quaternion getCurrentAngularPosition() override;

    /** @brief Returns the current angular speed at the current simulation time. */
    virtual inet::Quaternion getCurrentAngularVelocity() override;

    /**
     * Returns the current angular acceleration at the current simulation time.
     */
    virtual inet::Quaternion getCurrentAngularAcceleration() override;

    /** @brief Returns the constraint area maximum
     */
    virtual inet::Coord getConstraintAreaMax() const override;

    /** @brief Returns the constraint are minimum
     */
    virtual inet::Coord getConstraintAreaMin() const override;

    /**
     * Returns the current position at the given simulation time
     * @param time target simulation time in seconds
     * @return Position at given simulation time
     */
    virtual inet::Coord getPositionAtTime(double time);
};

}  // namespace estnet

#endif
