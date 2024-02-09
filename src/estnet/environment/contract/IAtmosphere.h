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

#ifndef ESTNET_ENVIRONMENT_CONTRACT_IATMOSPHERE_H_
#define ESTNET_ENVIRONMENT_CONTRACT_IATMOSPHERE_H_

#include <inet/common/geometry/common/Coord.h>
#include <inet/common/Units.h>

#include "common/ESTNETDefs.h"
#include "global_config.h"


class ESTNET_API IAtmosphere {
public:

    /**
     * This function calculates the length of intersection with the atmosphere
     * for the line of sight between the given points
     */
    inetu::km getDistanceThrughAtmosphere(inet::Coord point1, inet::Coord point2) = 0;

};

#endif /* ESTNET_ENVIRONMENT_CONTRACT_IATMOSPHERE_H_ */
