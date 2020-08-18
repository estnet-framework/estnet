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

#ifndef SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEECI_H_
#define SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEECI_H_

#include "PropStatePosition.h"

namespace estnet {

class ESTNET_API PropStateECI: public PropStatePosition {
public:
    /**
     * Returns the current orbit position of the object in ECI coordinates.
     * @param eci current orbit position of the object in ECI coordinates
     */
    virtual void asECI(cEci &eci) const override
    {
        eci = _eciState;
    }

    /**
     * Initializes the current orbit position from the given ECI coordinates.
     * @param eci ECI coordinates to initialize the orbit from
     */
    virtual void fromECI(cEci const &eci) override
    {
        _eciState = eci;
    }

    cEci _eciState;
};

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEECI_H_ */
