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

#ifndef __ESTNET_POSITIONPROPAGATORSGP4FILE_H_
#define __ESTNET_POSITIONPROPAGATORSGP4FILE_H_

#include "PositionPropagatorSGP4Base.h"

namespace estnet {

/**
 * Implements a SGP4 position propagator.
 * The TLE lines are given directly as arguments.
 */
class ESTNET_API PositionPropagatorSGP4File: public PositionPropagatorSGP4Base {
protected:
    /**
     * must be overriden by children to provide the two TLE lines
     */
    virtual void getTleLines(std::string &firstLine, std::string &secondLine)
            override;
};

}  // namespace estnet

#endif /* __ESTNET_POSITIONPROPAGATORSGP4FILE_H_ */
