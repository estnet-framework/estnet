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

#include "AttitudePropagatorQuaternion.h"

namespace estnet {

Register_Class(AttitudePropagatorQuaternion);

AttitudePropagatorQuaternion::AttitudePropagatorQuaternion() :
        AttitudePropagator() {
}

void AttitudePropagatorQuaternion::initialize() {
    // Do Propagator module initialization here...
    _minSAccuracy = par("dSAccMin");
}

void AttitudePropagatorQuaternion::handleMessage(omnetpp::cMessage *msg) {
    // We should not receive any messages directly!
    throw omnetpp::cRuntimeError(
            "Propagator received self-message, which it shouldn't!\n");
}

void AttitudePropagatorQuaternion::propagateState(const cJulian &targetTime,
        state_type &newState) {
    double dT = targetTime.spanSec(_lastUpdateTime);
    cQuaternion att, vel, acc;
    _currentState.attitudeAsQuaternion(att);
    _currentState.angularVelAsQuaternion(vel);
    _currentState.angularAccAsQuaternion(acc);

    //TODO attitude is only correctly calculated if angular acceleration is 0
    cQuaternion newAttitude = att + vel * att * (dT / 2);
    newAttitude.normalize();

    newState.attitudeFromQuaternion(newAttitude);
    newState.angularVelFromQuaternion(vel + acc * dT);
    newState.angularAccFromQuaternion(acc);
    _lastUpdateTime = targetTime;
    _currentState = newState;
}

bool AttitudePropagatorQuaternion::needsStateUpdate(const cJulian &targetTime) {
    // just check if we are below the accuracy threshold...
    double dT = targetTime.spanSec(_lastUpdateTime);
    cQuaternion vel, acc;
    _currentState.angularVelAsQuaternion(vel);
    _currentState.angularAccAsQuaternion(acc);
    // compute approximate distance travelled since last state update
    cEulerAngles tmpDist =
            ((vel * dT + acc * (pow(dT, 2.0) / 2.0))).toEulerAngles(true);
    cVector dDist = cVector(tmpDist.alpha.get(), tmpDist.beta.get(),
            tmpDist.gamma.get());
    return (dDist.x > _minSAccuracy || dDist.y > _minSAccuracy
            || dDist.y > _minSAccuracy);
}

}  // namespace estnet
