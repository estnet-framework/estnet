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

#ifndef SATMOBILITY_PROPAGATORS_ATTITUDE_PROPSTATEATTITUDE_H_
#define SATMOBILITY_PROPAGATORS_ATTITUDE_PROPSTATEATTITUDE_H_

#include "estnet/mobility/satellite/propagator/PropState.h"
#include "estnet/mobility/satellite/config_satm.h"

#ifdef USE_INET_EULER_ANGLES
#include <inet/common/geometry/common/EulerAngles.h>
#define cEulerAngles inet::EulerAngles
#endif

#ifdef USE_INET_QUATERNIONS
#include <inet/common/geometry/common/Quaternion.h>
#define cQuaternion inet::Quaternion
#else
#include "estnet/mobility/satellite/common/Quaternion.h"
#endif

namespace estnet {

/**
 * Interface for attitude state vectors
 */
class ESTNET_API PropStateAttitude: public PropState {
    template<typename tAttitudeState> friend class AttitudePropagator;

public:
    /**
     * Constructor of PropStateAttitude
     */
    PropStateAttitude() :
            PropState() {
    }

    /**
     * Get the current attitude in euler angle representation.
     * @param eAngles the current attitude in euler angle representation
     */
    virtual void attitudeAsEulerAngles(cEulerAngles &eAngles) const = 0;

    /**
     * Get the current attitude in quaternion representation.
     * @param quaternion the current attitude in quaternion representation
     */
    virtual void attitudeAsQuaternion(cQuaternion &quaternions) const = 0;

    /**
     * Get the current angular velocity in euler angle representation.
     * @param eAngles the current angular velocity in euler angle representation
     */
    virtual void angularVelAsEulerAngles(cEulerAngles &eAngles) const = 0;

    /**
     * Get the current angular velocity in quaternion representation.
     * @param quaternion the current angular velocity in quaternion representation
     */
    virtual void angularVelAsQuaternion(cQuaternion &quaternion) const = 0;

    /**
     * Get the current angular acceleration in euler angle representation.
     * @param eAngles the current angular acceleration in euler angle representation
     */
    virtual void angularAccAsEulerAngles(cEulerAngles &eAngles) const = 0;

    /**
     * Get the current angular acceleration in quaternion representation.
     * @param quaternion the current angular acceleration in quaternion representation
     */
    virtual void angularAccAsQuaternion(cQuaternion &quaternion) const = 0;

    /**
     * Set the current attitude from euler angles.
     * @param eAngles the current attitude in euler angles
     */
    virtual void attitudeFromEulerAngles(cEulerAngles const &eAngles) = 0;

    /**
     * Set the current attitude from quaternion.
     * @param quaternion the current attitude in quaternions
     */
    virtual void attitudeFromQuaternion(cQuaternion const &quaternion) = 0;

    /**
     * Set the current angular velocity from euler angles.
     * @param eAngles the current angular velocity in euler angles
     */
    virtual void angularVelFromEulerAngles(cEulerAngles const &eAngles) = 0;

    /**
     * Set the current angular velocity from quaternion.
     * @param quaternion the current angular velocity in quaternions
     */
    virtual void angularVelFromQuaternion(cQuaternion const &quaternion) = 0;

    /**
     * Set the current angular acceleration from euler angles.
     * @param eAngles the current angular acceleration in euler angles
     */
    virtual void angularAccFromEulerAngles(cEulerAngles const &eAngles) = 0;

    /**
     * Set the current angular acceleration from quaternion.
     * @param quaternion the current angular acceleration in quaternions
     */
    virtual void angularAccFromQuaternion(cQuaternion const &quaternion) = 0;
};

typedef std::shared_ptr<PropStateAttitude> tPropStateAttitude_Ptr; // Reference counting pointer to PropStateAttitude

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_ATTITUDE_PROPSTATEATTITUDE_H_ */
