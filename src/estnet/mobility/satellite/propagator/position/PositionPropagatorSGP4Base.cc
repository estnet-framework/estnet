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

#include "PositionPropagatorSGP4Base.h"

namespace estnet {

const char SGP4_TYPE_RUN = 'c'; // we don't want to be asked for times
const char SGP4_TYPE_INPUT = 'm';
const char SGP4_OPS_MODE = 'i';
const gravconsttype SGP4_GRAV_TYPE = wgs84;

void PositionPropagatorSGP4Base::initialize() {
    this->getTleLines(_firstLine, _secondLine);
    if (_firstLine.empty() || _secondLine.empty()) {
        throw omnetpp::cRuntimeError("No TLE found");
    }

    double startmfe, stopmfe, deltamin;
    twoline2rv((char*) _firstLine.c_str(), (char*) _secondLine.c_str(),
            SGP4_TYPE_RUN, SGP4_TYPE_INPUT, SGP4_OPS_MODE, SGP4_GRAV_TYPE,
            startmfe, stopmfe, deltamin, _satrec);
    // check for errors during initialization
    if (_satrec.error != 0) {
        EV_ERROR << "SGP4 Error " << _satrec.error << omnetpp::endl;
        throw omnetpp::cRuntimeError("SGP4 Error");
    }
}

void PositionPropagatorSGP4Base::handleMessage(omnetpp::cMessage *msg) {
}

bool PositionPropagatorSGP4Base::needsStateUpdate(cJulian const &targetTime) {
    return true;
}

void PositionPropagatorSGP4Base::propagateState(cJulian const &targetTime,
        state_type &newState) {
    cEci temp = GetState(targetTime);
    cEci state;
    state.setPos(temp.getPos());
    state.setVel(temp.getVel());
    state.setDate(targetTime);
    state.setUnitsM();

    newState.fromECI(state);
    _lastUpdateTime = targetTime;
    _currentState = newState;
}

double PositionPropagatorSGP4Base::getOrbitalPeriod() const {
    return 1440.0 / _satrec.no / (1440.0 / (2.0 * M_PI)) * 60.0;
}

double PositionPropagatorSGP4Base::getOrbitalRadius(
        cJulian const &targetTime) const {
    cEci lastState;
    _currentState.asECI(lastState);
    return lastState.getPos().length() / 1000.0;
}

cEci PositionPropagatorSGP4Base::GetState(cJulian const &targetTime) const {
    cEci ret;
    double pos[3];
    double vel[3];

    cJulian epochSGP4 = cJulian(_satrec.epochyr + 2000, _satrec.epochdays);
    double time_in_minutes = targetTime.spanMin(epochSGP4);

    // hacking around that _satrec is const because the method is const
    elsetrec satrec_cpy = _satrec;
    sgp4(SGP4_GRAV_TYPE, satrec_cpy, time_in_minutes, pos, vel);
    memcpy((elsetrec*) &_satrec, &satrec_cpy, sizeof(elsetrec));

    // check for errors during propagation
    if (_satrec.error != 0) {
        EV_ERROR << "SGP4 Error " << _satrec.error << omnetpp::endl;
        throw omnetpp::cRuntimeError("SGP4 Error");
    }
    ret.setPos(cVector(pos[0] * 1000, pos[1] * 1000, pos[2] * 1000));
    ret.setVel(cVector(vel[0] * 1000, vel[1] * 1000, vel[2] * 1000));
    ret.setUnitsM();
    ret.setDate(targetTime);

    return ret;
}

}  // namespace estnet
