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

#ifndef __UTILS_APSK_SCALAR_TRANSMITTER_WITH_8PSK_H__
#define __UTILS_APSK_SCALAR_TRANSMITTER_WITH_8PSK_H__

#include <inet/physicallayer/apskradio/packetlevel/ApskScalarTransmitter.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * APSKScalarTransmitter with support for 8PSK modulation
 */
class ESTNET_API APSKScalarTransmitterWith8PSK: public inet::physicallayer::ApskScalarTransmitter {
    /**
     * Overrides initialization of ApskScalarTransmitter, as it does not allow
     * other modulations
     */
    virtual void initialize(int stage) override;
};

}  // namespace estnet

#endif
