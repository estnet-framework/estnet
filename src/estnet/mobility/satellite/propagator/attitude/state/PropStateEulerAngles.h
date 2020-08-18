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

#ifndef SATMOBILITY_PROPAGATORS_ATTITUDE_PROPSTATEEULERANGLES_H_
#define SATMOBILITY_PROPAGATORS_ATTITUDE_PROPSTATEEULERANGLES_H_

#include "PropStateAttitude.h"
#include "estnet/mobility/satellite/config_satm.h"

namespace estnet {

class ESTNET_API PropStateEulerAngles: public PropStateAttitude {
public:
    PropStateEulerAngles();

    /**
     * Get the current attitude in euler angle representation.
     * @param eAngles the current attitude in euler angle representation
     */
    virtual void attitudeAsEulerAngles(cEulerAngles &eAngles) const;

    /**
     * Get the current attitude in quaternion representation.
     * @param quaternion the current attitude in quaternion representation
     */
    virtual void attitudeAsQuaternion(cQuaternion &quaternions) const;

    /**
     * Get the current angular velocity in euler angle representation.
     * @param eAngles the current angular velocity in euler angle representation
     */
    virtual void angularVelAsEulerAngles(cEulerAngles &eAngles) const;

    /**
     * Get the current angular velocity in quaternion representation.
     * @param quaternion the current angular velocity in quaternion representation
     */
    virtual void angularVelAsQuaternion(cQuaternion &quaternion) const;

    /**
     * Get the current angular acceleration in euler angle representation.
     * @param eAngles the current angular acceleration in euler angle representation
     */
    virtual void angularAccAsEulerAngles(cEulerAngles &eAngles) const;

    /**
     * Get the current angular acceleration in quaternion representation.
     * @param quaternion the current angular acceleration in quaternion representation
     */
    virtual void angularAccAsQuaternion(cQuaternion &quaternion) const;

    /**
     * Set the current attitude from euler angles.
     * @param eAngles the current attitude in euler angles
     */
    virtual void attitudeFromEulerAngles(cEulerAngles const &eAngles);

    /**
     * Set the current attitude from quaternion.
     * @param quaternion the current attitude in quaternions
     */
    virtual void attitudeFromQuaternion(cQuaternion const &quaternion);

    /**
     * Set the current angular velocity from euler angles.
     * @param eAngles the current angular velocity in euler angles
     */
    virtual void angularVelFromEulerAngles(cEulerAngles const &eAngles);

    /**
     * Set the current angular velocity from quaternion.
     * @param quaternion the current angular velocity in quaternions
     */
    virtual void angularVelFromQuaternion(cQuaternion const &quaternion);

    /**
     * Set the current angular acceleration from euler angles.
     * @param eAngles the current angular acceleration in euler angles
     */
    virtual void angularAccFromEulerAngles(cEulerAngles const &eAngles);

    /**
     * Set the current angular acceleration from quaternion.
     * @param quaternion the current angular acceleration in quaternions
     */
    virtual void angularAccFromQuaternion(cQuaternion const &quaternion);

private:
    inet::EulerAngles _attitude;
    inet::EulerAngles _angularVelocity;
    inet::EulerAngles _angularAcceleration;
};

}  // namespace estnet

#endif
