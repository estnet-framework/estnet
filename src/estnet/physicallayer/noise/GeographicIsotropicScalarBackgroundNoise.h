//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#ifndef __GEOGRAPHIC_ISOTROPIC_SCALAR_BACKGROUND_NOISE_H
#define __GEOGRAPHIC_ISOTROPIC_SCALAR_BACKGROUND_NOISE_H

#include <omnetpp.h>

#include "inet/physicallayer/backgroundnoise/IsotropicScalarBackgroundNoise.h"
#include "inet/physicallayer/contract/packetlevel/IBackgroundNoise.h"

#include "estnet/common/ESTNETDefs.h"
#include "estnet/common/interpolation/SphericalBilinearInterpolation.h"
#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

class ESTNET_API GeographicIsotropicScalarBackgroundNoise
    : public inet::physicallayer::IsotropicScalarBackgroundNoise {
protected:
  SphericalBilinearInterpolation interpolation;

protected:
  virtual void initialize(int stage) override;

public:
  GeographicIsotropicScalarBackgroundNoise();

  virtual const inet::physicallayer::INoise *
  computeNoise(const inet::physicallayer::IListening *listening) const override;

private:
  IEarthModel *earthModel;
};

} // namespace estnet

#endif // ifndef __GEOGRAPHIC_ISOTROPIC_SCALAR_BACKGROUND_NOISE_H
