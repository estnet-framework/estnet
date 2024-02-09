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
 * A JSON file with different TLEs for different times is read in.
 * When the position is requested, first the right TLE is selected based on the
 * requested time and then the position is calculated as in PositionPropagatorSGP4Base.
 * Some of the functions need to be overriden and copied from PositionPropagatorSGP4Base
 * to implement this functionality.
 */
class ESTNET_API PositionPropagatorSGP4List: public PositionPropagatorSGP4Base {
protected:
    /**
     * must be overriden by children but is not used for this class
     */
    virtual void getTleLines(std::string &firstLine, std::string &secondLine)
            override;
    /**
     * Initializes all the orbit-parameters and the 3D anmimation.
     * Schedules the first initial movement
     * @param stage of the initialization (2 steps needed)
     */
    virtual void initialize() override;

    /**
     * Propagates the satellite state starting from the current state until the
     * targetTime point in time.
     * @param targetTime point in time for which the state should be propagated
     * @param newState new propagated state
     * Note: Must be implemented by the respective propagators
     */
    virtual void propagateState(cJulian const &targetTime, state_type &newState)
            override;

    // returns position[m] and velocity [m/s] at given time
    cEci GetState(cJulian const &targetTime) const;

    /**
     * Returns the orbital period of the current orbit.
     * Implementation for the IPositionPropagator interface
     * @return orbital period in seconds
     */
    virtual double getOrbitalPeriod() const override;

private:
    std::vector<elsetrec> _satrec; // vector of all TLEs
    int currentElement = -1; // index of the TLE vector for the last requested position
};


}  // namespace estnet

#endif /* __ESTNET_POSITIONPROPAGATORSGP4FILE_H_ */
