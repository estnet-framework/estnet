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

#ifndef SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEPOSITION_H_
#define SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEPOSITION_H_

#include <memory>

#include "estnet/mobility/satellite/propagator/PropState.h"
#include "estnet/mobility/satellite/common/cEci.h"

namespace estnet {

class ESTNET_API PropStatePosition: public PropState {
    template<typename tPositionState> friend class PositionPropagator;

public:
    /**
     * Converts the current state to ECI coordinates
     * @param eci eci coordinates of the current state
     */
    virtual void asECI(cEci &eci) const = 0;

    /**
     * Converts the given eci coordinates to the current state.
     * @param eci eci coordiantes to convert into the current state.
     */
    virtual void fromECI(cEci const &eci) = 0;

private:
};

typedef std::shared_ptr<PropStatePosition> tPropStatePosition_Ptr; // Reference counting pointer to PropStatePosition

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_POSITION_PROPSTATEPOSITION_H_ */
