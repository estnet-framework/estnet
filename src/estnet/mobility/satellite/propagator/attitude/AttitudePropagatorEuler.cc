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

#include "AttitudePropagatorEuler.h"

namespace estnet {

Register_Class(AttitudePropagatorEuler);

AttitudePropagatorEuler::AttitudePropagatorEuler() :
        AttitudePropagator() {
}

void AttitudePropagatorEuler::initialize() {
    // Do Propagator module initialization here...
    _minSAccuracy = par("dSAccMin");
}

void AttitudePropagatorEuler::handleMessage(omnetpp::cMessage *msg) {
    // We should not receive any messages directly!
    throw omnetpp::cRuntimeError(
            "Propagator received self-message, which it shouldn't!\n");
}

void AttitudePropagatorEuler::propagateState(const cJulian &targetTime,
        state_type &newState) {
    double dT = targetTime.spanSec(_lastUpdateTime);
    cEulerAngles att, vel, acc;
    _currentState.attitudeAsEulerAngles(att);
    _currentState.angularVelAsEulerAngles(vel);
    _currentState.angularAccAsEulerAngles(acc);

    newState.attitudeFromEulerAngles(
            att + vel * dT + acc * (1.0 / 2.0) * pow(dT, 2.0));
    newState.angularVelFromEulerAngles(vel + acc * dT);
    newState.angularAccFromEulerAngles(acc);
    _lastUpdateTime = targetTime;
    _currentState = newState;
}

bool AttitudePropagatorEuler::needsStateUpdate(const cJulian &targetTime) {
    // just check if we are below the accuracy threshold...
    double dT = targetTime.spanSec(_lastUpdateTime);
    cEulerAngles tmpAVel;
    cEulerAngles tmpAAcc;
    _currentState.angularVelAsEulerAngles(tmpAVel);
    _currentState.angularAccAsEulerAngles(tmpAAcc);
    cVector tmpAVelv = cVector(tmpAVel.alpha.get(), tmpAVel.beta.get(),
            tmpAVel.gamma.get());
    cVector tmpAAccv = cVector(tmpAAcc.alpha.get(), tmpAAcc.beta.get(),
            tmpAAcc.gamma.get());
    // compute approximate distance travelled since last state update
    cVector dDist = tmpAAccv * (1.0 / 2.0) * pow(dT, 2.0) + tmpAVelv * dT;
    return (dDist.x > _minSAccuracy || dDist.y > _minSAccuracy
            || dDist.y > _minSAccuracy);
}

}  // namespace estnet
