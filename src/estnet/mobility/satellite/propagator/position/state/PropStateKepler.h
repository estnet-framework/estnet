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

#ifndef SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEKEPLER_H_
#define SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEKEPLER_H_

#include "PropStatePosition.h"
#include "estnet/mobility/satellite/config_satm.h"
#include "estnet/mobility/satellite/common/cEci.h"

#ifdef USE_INET_COORDS
#include <inet/common/geometry/common/Coord.h>
#define cVector inet::Coord
#endif

namespace estnet {

typedef struct _sKeplerParams {
    double _e; // Eccentricity
    double _a; // Semimajor Axis
    double _i; // Inclination
    double _W; // Longitude of the ascending node
    double _w; // Argument of periapsis
    double _v; // true anomaly at current time stamp
} tKeplerParams;

class ESTNET_API PropStateKepler: public PropStatePosition {
public:
    /**
     * Returns the current orbit position of the object in ECI coordinates.
     * @param eci current orbit position of the object in ECI coordinates
     */
    virtual void asECI(cEci &eci) const override;

public:
    /**
     * Returns the current kepler parameters of this state.
     * @return the current kepler parameters of this state
     */
    virtual tKeplerParams getKeplerParameters() const {
        return _KeplerP;
    }

    /**
     * Initializes the current orbit position from the given ECI coordinates.
     * @param eci ECI coordinates to initialize the orbit from
     */
    virtual void fromECI(cEci const &eci) override;

    tKeplerParams _KeplerP; // Kepler parameters of this state at the current time stamp
};

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEKEPLER_H_ */
