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

#ifndef __UTILS_APSK_SCALAR_RADIO_EXTRA_STATISTICS_H__
#define __UTILS_APSK_SCALAR_RADIO_EXTRA_STATISTICS_H__

#include <inet/physicallayer/apskradio/packetlevel/ApskRadio.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

class ESTNET_API APSKScalarRadioExtraStatistics: public inet::physicallayer::ApskRadio {
protected:
    virtual inet::units::values::b computePaddingLength(
            inet::units::values::b length,
            const inet::physicallayer::ConvolutionalCode *forwardErrorCorrection,
            const inet::physicallayer::ApskModulationBase *modulation) const
                    override {
        return inet::units::values::b(0);
    }
    ;
};

}  // namespace estnet

#endif
