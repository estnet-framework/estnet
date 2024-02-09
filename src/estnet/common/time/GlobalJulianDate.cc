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

#include "estnet/common/time/GlobalJulianDate.h"

#include <string>
#include <fstream>

#include "estnet/mobility/satellite/common/sgp4/sgp4io.h"

namespace estnet {

static GlobalJulianDate *instance = nullptr;

Define_Module(GlobalJulianDate);

const char SGP4_TYPE_RUN = 'c'; // we don't want to be asked for times
const char SGP4_TYPE_INPUT = 'm';
const char SGP4_OPS_MODE = 'i';
const gravconsttype SGP4_GRAV_TYPE = wgs84;

GlobalJulianDate::GlobalJulianDate() {
    if (instance != nullptr) {
        throw omnetpp::cRuntimeError(
                "There can be only one GlobalJulianDate instance in the network");
    }
    instance = this;
}

GlobalJulianDate::~GlobalJulianDate() {
    instance = nullptr;
}

const GlobalJulianDate& GlobalJulianDate::getInstance() {
    return *instance;
}

const GlobalJulianDate* GlobalJulianDate::getInstancePtr() {
    return instance;
}

void GlobalJulianDate::initialize() {
    std::string firstLine, secondLine;
    bool hasTLE = this->getTleLines(firstLine, secondLine);
    if ((firstLine.empty() || secondLine.empty()) || !hasTLE) {
        const char *dateString = par("simulationStart").stringValue();

        if (std::string(dateString).empty()) {
            throw omnetpp::cRuntimeError(
                    "No time reference given! Please give date or TLE in GlobalJulianDate.");

        }
        int y, M, d, h, m, tzh = 0, tzm = 0;
        float s;
        if (6
                < sscanf(dateString, "%d-%d-%dT%d:%d:%f%d:%dZ", &y, &M, &d, &h,
                        &m, &s, &tzh, &tzm)) {
            if (tzh < 0) {
                tzm = -tzm;
            }
        }

        this->_simStart = cJulian(y, M, d, h, m, s);
        _simStart.addHour(-tzh);
        _simStart.addMin(-tzm);

    } else {
        elsetrec satrec;

        double startmfe, stopmfe, deltamin;
        twoline2rv((char*) firstLine.c_str(), (char*) secondLine.c_str(),
                SGP4_TYPE_RUN, SGP4_TYPE_INPUT, SGP4_OPS_MODE, SGP4_GRAV_TYPE,
                startmfe, stopmfe, deltamin, satrec);
        // check for errors during initialization
        if (satrec.error != 0) {
            EV_ERROR << "SGP4 Error " << satrec.error << omnetpp::endl;
            throw omnetpp::cRuntimeError("TLE Error: ");
        }
        this->_simStart = cJulian(satrec.epochyr + 2000, satrec.epochdays);

    }

}

cJulian GlobalJulianDate::currentSimTime() const {
    return simTime2JulianDate(SIMTIME_DBL(omnetpp::simTime()));
}

cJulian GlobalJulianDate::simTime2JulianDate(double simTime) const {
    cJulian tmpJulian(this->_simStart);
    tmpJulian.addSec(simTime);
    return tmpJulian;
}

double GlobalJulianDate::julianDate2SimTime(const cJulian &simTime) const {
    return simTime.spanSec(this->_simStart);
}

bool GlobalJulianDate::getTleLines(std::string &firstLine,
        std::string &secondLine) {
    std::string tleFileName = this->par("tleFile").stringValue();
    std::ifstream tleFile;
    tleFile.open(tleFileName);
    if (!tleFile.is_open()) {
        return false;
    }
    // skip first line
    getline(tleFile, firstLine);

    getline(tleFile, firstLine);
    getline(tleFile, secondLine);
    return true;
}

} //namespace
