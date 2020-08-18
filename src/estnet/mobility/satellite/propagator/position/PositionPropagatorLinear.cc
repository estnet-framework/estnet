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

#include "PositionPropagatorLinear.h"

#include <omnetpp.h>

namespace estnet {

Register_Class(PositionPropagatorLinear);

void PositionPropagatorLinear::initialize() {
    // Do Propagator module initialization here...
    _minSAccuracy = par("dSAccMin");
}

void PositionPropagatorLinear::handleMessage(omnetpp::cMessage *msg) {
    // We should not receive any messages directly!
    throw omnetpp::cRuntimeError(
            "Propagator received self-message, which it shouldn't!\n");
}

void PositionPropagatorLinear::propagateState(const cJulian &targetTime,
        state_type &newState) {
    // TODO: perform linear state update
}

bool PositionPropagatorLinear::needsStateUpdate(const cJulian &targetTime) {
    // just check if we are below the accuracy threshold...
    double dT = targetTime.spanSec(_lastUpdateTime);
    cEci tmpEci;
    _currentState.asECI(tmpEci);
    // compute approximate distance travelled since last state update (assuming
    // constant velocity!)
    cVector dDist = tmpEci.getVel() * dT;
    return (dDist.x > _minSAccuracy || dDist.y > _minSAccuracy
            || dDist.y > _minSAccuracy);
}

double PositionPropagatorLinear::getOrbitalPeriod() const {
    // TODO: return a matching 'orbit period'
    return 0;
}

double PositionPropagatorLinear::getOrbitalRadius(
        cJulian const &targetTime) const {
    // TODO: return a matching 'orbit radius'
    return 0;
}

}  // namespace estnet
