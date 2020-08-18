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

#ifndef __UTILS_SCALAR_ANALOG_MODEL_ANTENNA_FIX_H__
#define __UTILS_SCALAR_ANALOG_MODEL_ANTENNA_FIX_H__

#include <inet/physicallayer/analogmodel/packetlevel/ScalarAnalogModel.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * An implementation of the ~ScalarAnalogModel which fixes
 * transmission and reception angle calculations when nodes
 * have multiple degrees of rotational freedom.
 */
class ESTNET_API ScalarAnalogModelAntennaFix: public inet::physicallayer::ScalarAnalogModel {
protected:
    /** @brief calculates the transmission angle using Quaternions*/
    virtual inet::Quaternion computeTransmissionDirection(
            const inet::physicallayer::ITransmission *transmission,
            const inet::physicallayer::IArrival *arrival) const;
    /** @brief calculates the reception angle using Quaternions*/
    virtual inet::Quaternion computeReceptionDirection(
            const inet::physicallayer::ITransmission *transmission,
            const inet::physicallayer::IArrival *arrival) const;

public:
    /** @brief calculates signal power at the receiving node */
    virtual inet::units::values::W
    computeReceptionPower(const inet::physicallayer::IRadio *receiverRadio,
            const inet::physicallayer::ITransmission *transmission,
            const inet::physicallayer::IArrival *arrival) const;
};

}  // namespace estnet

#endif
