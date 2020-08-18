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

#ifndef __UTILS_PSK8_MODULATION_H__
#define __UTILS_PSK8_MODULATION_H__

#include <inet/physicallayer/base/packetlevel/ApskModulationBase.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * Implementes BER & SER calculations for the 8PSK modulation. (Class names cannot begin with numbers)
 */
class ESTNET_API PSK8Modulation: public inet::physicallayer::ApskModulationBase {
public:
    PSK8Modulation() :
            inet::physicallayer::ApskModulationBase(
                    new std::vector<inet::physicallayer::ApskSymbol>()) {
    }

    /** @brief Prints this object to the provided output stream */
    virtual std::ostream& printToStream(std::ostream &stream, int level) const
            override {
        return stream << "8PSKModulation";
    }

    /**
     * Returns the symbol error rate as a function of the signal to noise
     * and interference ratio, the bandwidth, and the gross (physical) bitrate.
     */
    virtual double calculateSER(double snir, inet::Hz bandwidth,
            inet::bps bitrate) const override;
    /**
     * Returns the bit error rate as a function of the signal to noise and
     * interference ratio, the bandwidth, and the gross (physical) bitrate.
     */
    virtual double calculateBER(double snir, inet::Hz bandwidth,
            inet::bps bitrate) const override;

};

}  // namespace estnet

#endif
