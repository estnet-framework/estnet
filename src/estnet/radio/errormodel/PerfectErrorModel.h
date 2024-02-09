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

#ifndef __UTILS__PERFECT_ERROR_MODEL_H__
#define __UTILS__PERFECT_ERROR_MODEL_H__

#include <inet/physicallayer/base/packetlevel/ErrorModelBase.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * Implementation of ~IErrorModel that always
 * returns 0% error probability.
 */
class ESTNET_API PerfectErrorModel: public inet::physicallayer::ErrorModelBase {
public:
    /** @brief Prints this object to the provided output stream */
    virtual std::ostream& printToStream(std::ostream &stream, int level) const
            override;

    /**
     * Returns the packet error rate based on SNIR, modulation, FEC encoding
     * and any other physical layer characteristics.
     */
    virtual double computePacketErrorRate(
            const inet::physicallayer::ISnir *snir,
            inet::physicallayer::IRadioSignal::SignalPart part) const override;

    /**
     * Returns the bit error rate based on SNIR, modulation, FEC encoding
     * and any other physical layer characteristics.
     */
    virtual double computeBitErrorRate(const inet::physicallayer::ISnir *snir,
            inet::physicallayer::IRadioSignal::SignalPart part) const override;

    /**
     * Returns the symbol error rate based on SNIR, modulation, and any other
     * physical layer characteristics.
     */
    virtual double computeSymbolErrorRate(
            const inet::physicallayer::ISnir *snir,
            inet::physicallayer::IRadioSignal::SignalPart part) const override;
};

}  // namespace estnet

#endif

