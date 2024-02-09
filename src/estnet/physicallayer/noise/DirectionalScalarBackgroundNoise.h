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

#ifndef __DIRECTIONAL_SCALAR_BACKGROUND_NOISE_H__
#define __DIRECTIONAL_SCALAR_BACKGROUND_NOISE_H__

#include <omnetpp.h>

#include <inet/physicallayer/contract/packetlevel/IBackgroundNoise.h>
#include <inet/physicallayer/backgroundnoise/IsotropicScalarBackgroundNoise.h>

#include "estnet/common/ESTNETDefs.h"

namespace estnet {
/*
 * BackgroundNoise class that accounts for the direction the satellite is pointing
 * and how much of its viewing cone is looking at the earth or space. Based on the
 * ratio between earth and space and the temperature of each of them compute the noise power.
 */
class ESTNET_API DirectionalScalarBackgroundNoise: public inet::physicallayer::IsotropicScalarBackgroundNoise {
public:
    DirectionalScalarBackgroundNoise();
    /*
     * Computes the percentage of the earth inside the beamwidth of the antenna
     */
    virtual const double computeEarthInFOV(inet::Coord SatPosition,
            inet::Quaternion SatOrientation, double beamWidth) const;
    /*
     * Computes the noise of the antenna by determining the beamwidth of the antenna in use
     * and the fraction of space and earth inside the FOV of the antenna.
     */
    virtual const double computeAntennaNoise(
            const inet::physicallayer::IListening *listening) const;
    virtual const inet::physicallayer::INoise* computeNoise(
            const inet::physicallayer::IListening *listening) const override;

protected:
    virtual void initialize(int stage) override;

    double receiverNoiseTemp;
    double t_Earth;
    double t_Space;

};

}  //estnet

#endif // ifndef __DIRECTIONAL_SCALAR_BACKGROUND_NOISE_H__
