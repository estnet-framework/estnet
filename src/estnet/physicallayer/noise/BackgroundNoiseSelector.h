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

#ifndef ESTNET_NOISE_BACKGROUNDNOISESELECTOR_H_
#define ESTNET_NOISE_BACKGROUNDNOISESELECTOR_H_

#include <omnetpp.h>

#include "inet/physicallayer/contract/packetlevel/IBackgroundNoise.h"
#include "inet/physicallayer/contract/packetlevel/IAntenna.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"

#include "estnet/common/ESTNETDefs.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/terrestrial/StaticTerrestrialMobility.h"

namespace estnet {
/*
 * This class is just used to select a noise model based on which node it belongs to.
 * It can distinguish between satellite and groundstation based on the type of mobility used.
 */
class ESTNET_API BackgroundNoiseSelector: public cModule,
        public inet::physicallayer::IBackgroundNoise {
protected:
    virtual void initialize(int stage) override;

public:
    BackgroundNoiseSelector();

    virtual const inet::physicallayer::INoise* computeNoise(
            const inet::physicallayer::IListening *listening) const override;
};

} //estnet

#endif /* ESTNET_NOISE_BACKGROUNDNOISESELECTOR_H_ */
