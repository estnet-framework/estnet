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

#ifndef __ANTENNAS_YAGI_ANTENNA_H__
#define __ANTENNAS_YAGI_ANTENNA_H__

#include <inet/common/Units.h>
#include <inet/physicallayer/base/packetlevel/AntennaBase.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {

/**
 * Base class for an antenna with its own mobility
 */
class ESTNET_API AntennaBaseWithOrientation: public inet::physicallayer::AntennaBase {
protected:
    /** @brief initialization method */
    virtual void initialize(int stage) override;
};

}  // namespace estnet

#endif
