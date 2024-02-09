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

#ifndef __ESTNET_ATMOSPHERICPATHLOSS_H_
#define __ESTNET_ATMOSPHERICPATHLOSS_H_

#include <inet/physicallayer/pathloss/FreeSpacePathLoss.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * This path loss class extends the free space path loss
 * with losses, that appear in the atmosphere
 */
class ESTNET_API AtmosphericPathLoss: public inet::physicallayer::FreeSpacePathLoss {
public:
    /**
     * Returns the loss factor for the provided transmission and arrival.
     * The value is in the range [0, 1] where 1 means no loss at all and 0
     * means all power is lost.
     */
    virtual double computePathLoss(
            const inet::physicallayer::ITransmission *transmission,
            const inet::physicallayer::IArrival *arrival) const override;

    /**
     * Returns the loss factor as a function of propagation speed, carrier
     * frequency and distance. The value is in the range [0, 1] where 1 means
     * no loss at all and 0 means all power is lost.
     */
    virtual double computePathLoss(inet::mps propagationSpeed,
            inet::Hz frequency, inet::m distance) const override;

    /**
     * Returns the range for the given loss factor. The value is in the range
     * [0, +infinity) or NaN if unspecified.
     */
    virtual inet::m computeRange(inet::mps propagationSpeed, inet::Hz frequency,
            double loss) const override;

protected:
    /**
     *   Initialze Module, read in the set parameters
     */
    virtual void initialize(int stage);

    double _fogAttenuation;
    double _snowAttenuation;

    /*Atmospheric Loss [dB]
    Polarization loss [dB]
    Ionospheric loss [dB]
    Atmospheric Reflection [dB]*/
};

} //namespace

#endif
