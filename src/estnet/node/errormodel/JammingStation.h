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

#ifndef NODES_JAMMINGSTATION_H_
#define NODES_JAMMINGSTATION_H_

#include <inet/common/Units.h>
#include <inet/common/geometry/common/Coord.h>

#include "../groundstation/GroundLabel.h"
#include "estnet/environment/contract/IEarthModel.h"
#include "estnet/environment/earthmodel/EarthModelFactory.h"

namespace estnet {

/**
 *  class for jammer, that jam nodes containing a JammedPacketHandler
 *  if they are in range of the minimum elevation
 *  @see JammedPacketHandler
 */
class ESTNET_API JammingStation: public GroundLabel {
private:
    double probability;
    inetu::deg latitude;
    inetu::deg longitude;
    inetu::deg minElevation;
    inetu::Hz centerFrequency;
    inetu::Hz bandwidth;
    IEarthModel *earthModel;

public:
    /**
     * calculate if the coordinates given are within the jamming elevation
     * @param coordinates: given ECI coordinates of the object, for which the
     * elevation is calculated and compared to the given threshold
     * @return bool: returns true, when the object at the given coordinate is
     *         in elevation range
     */
    bool isInJammingRange(inet::Coord &coordinates);

    /**
     * Returns the propability of jamming the node, when the node is in elevation
     * range
     * @return double: propability in range of 0.0 to 1.0
     */
    double getProbability();

    /**
     * Multi-stage initialization of module
     */
    virtual void initialize(int stage);

    /**
     * Access to the center frequency of the jammed spectrum
     * @return Hz: center frequency of jammed spectrum in Hertz
     */
    inetu::Hz getCenterFrequency();

    /**
     * Access to the jammed bandwidth
     * @return Hz: Bandwidth in Hertz
     */
    inetu::Hz getBandwidth();
};

}  // namespace estnet

#endif /* NODES_JAMMINGSTATION_H_ */
