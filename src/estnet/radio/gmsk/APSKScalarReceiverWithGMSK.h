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

#ifndef __UTILS_APSK_SCALAR_RECEIVER_WITH_GMSK_H__
#define __UTILS_APSK_SCALAR_RECEIVER_WITH_GMSK_H__

#include <inet/physicallayer/apskradio/packetlevel/ApskScalarTransmission.h>
#include <inet/physicallayer/common/packetlevel/BandListening.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskScalarReceiver.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * APSKScalarReceiver with support for GMSK modulation
 */
class ESTNET_API APSKScalarReceiverWithGMSK: public inet::physicallayer::ApskScalarReceiver {
    virtual void initialize(int stage) override;

    virtual bool computeIsReceptionPossible(
            const inet::physicallayer::IListening *listening,
            const inet::physicallayer::ITransmission *transmission) const
                    override;
    virtual bool computeIsReceptionPossible(
            const inet::physicallayer::IListening *listening,
            const inet::physicallayer::IReception *reception,
            inet::physicallayer::IRadioSignal::SignalPart part) const override;

private:
    static omnetpp::simsignal_t noisePower;
};

}  // namespace estnet

#endif
