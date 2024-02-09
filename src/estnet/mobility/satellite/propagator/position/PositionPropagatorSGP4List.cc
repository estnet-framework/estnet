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

#include "PositionPropagatorSGP4List.h"

#include <fstream>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

using namespace rapidjson;

namespace estnet {

Register_Class(PositionPropagatorSGP4List);

const char SGP4_TYPE_RUN = 'c'; // we don't want to be asked for times
const char SGP4_TYPE_INPUT = 'm';
const char SGP4_OPS_MODE = 'i';
const gravconsttype SGP4_GRAV_TYPE = wgs84;

void PositionPropagatorSGP4List::getTleLines(std::string &firstLine,
        std::string &secondLine) {
}

void PositionPropagatorSGP4List::initialize() {
    std::string tleFileName = this->par("tleFile").stringValue();
    std::ifstream tleFile;
    tleFile.open(tleFileName);
    if (!tleFile.is_open()) {
        throw omnetpp::cRuntimeError("Could not open TLE file!");
    }

    IStreamWrapper isw(tleFile);
    Document d;
    d.ParseStream(isw);
    // go through all TLEs in the file
    for (auto &v : d.GetArray()) {
        elsetrec satrec;
        double startmfe, stopmfe, deltamin;
        twoline2rv((char*) v["TLE_LINE1"].GetString(),
                (char*) v["TLE_LINE2"].GetString(), SGP4_TYPE_RUN,
                SGP4_TYPE_INPUT, SGP4_OPS_MODE, SGP4_GRAV_TYPE, startmfe,
                stopmfe, deltamin, satrec);
        // check for errors during initialization
        if (satrec.error != 0) {
            EV_ERROR << "SGP4 Error " << satrec.error << omnetpp::endl;
            throw omnetpp::cRuntimeError("SGP4 Error");
        }

        _satrec.push_back(satrec);
    }
}

void PositionPropagatorSGP4List::propagateState(cJulian const &targetTime,
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

double PositionPropagatorSGP4List::getOrbitalPeriod() const {
    return 1440.0 / _satrec[currentElement].no / (1440.0 / (2.0 * M_PI)) * 60.0;
}

cEci PositionPropagatorSGP4List::GetState(cJulian const &targetTime) const {
    cEci ret;
    double pos[3];
    double vel[3];

    // find out which TLE should be used based on the targetTime
    if (currentElement != _satrec.size() - 1) {
        // ensure that also times in the past to previous request are handled
        // therefore, go through all elements every time
        for (int i = 0; i < _satrec.size() - 1; ++i) {
            cJulian epochSGP4 = cJulian(_satrec[i].epochyr + 2000,
                    _satrec[i].epochdays);
            cJulian epochSGP4Next = cJulian(_satrec[i + 1].epochyr + 2000,
                    _satrec[i + 1].epochdays);
            if (epochSGP4 < targetTime && targetTime < epochSGP4Next) {
                memcpy((int*) &currentElement, &i, sizeof(int));
                break;
            } else if (i == _satrec.size() - 2){ // use last element if targetTime is greater then it
                int tmp = i + 1;
                memcpy((int*) &currentElement, &(tmp), sizeof(int));
            }
        }
    }

    cJulian epochSGP4 = cJulian(_satrec[currentElement].epochyr + 2000,
            _satrec[currentElement].epochdays);
    double time_in_minutes = targetTime.spanMin(epochSGP4);
    // hacking around that _satrec is const because the method is const
    elsetrec satrec_cpy = _satrec[currentElement];
    sgp4(SGP4_GRAV_TYPE, satrec_cpy, time_in_minutes, pos, vel);
    memcpy((elsetrec*) &_satrec[currentElement], &satrec_cpy, sizeof(elsetrec));

    // check for errors during propagation
    if (_satrec[currentElement].error != 0) {
        EV_ERROR << "SGP4 Error " << _satrec[currentElement].error
                        << omnetpp::endl;
        throw omnetpp::cRuntimeError("SGP4 Error");
    }
    ret.setPos(cVector(pos[0] * 1000, pos[1] * 1000, pos[2] * 1000));
    ret.setVel(cVector(vel[0] * 1000, vel[1] * 1000, vel[2] * 1000));
    ret.setUnitsM();
    ret.setDate(targetTime);

    return ret;
}

}  // namespace estnet
