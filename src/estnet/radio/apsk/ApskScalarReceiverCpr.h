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

#ifndef __ESTNET_APSKSCALARRECEIVERCPR_H_
#define __ESTNET_APSKSCALARRECEIVERCPR_H_

#include <inet/physicallayer/apskradio/packetlevel/ApskScalarReceiver.h>
#include "estnet/common/ESTNETDefs.h"

using namespace omnetpp;

namespace estnet {

/**
 * ApskScalarReceiver class with contact plan routing SNIR added
 * @see ApskScalarReceiver
 */
class ESTNET_API ApskScalarReceiverCpr: public inet::physicallayer::ApskScalarReceiver {
protected:
    virtual void initialize(int stage) override;
};

} //namespace

#endif
