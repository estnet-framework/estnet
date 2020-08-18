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

namespace estnet {

static GlobalJulianDate* instance = nullptr;

Define_Module(GlobalJulianDate);

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
    const char * dateString = par("simulationStart").stringValue();
    int y, M, d, h, m, tzh = 0, tzm = 0;
    float s;
    if (6
            < sscanf(dateString, "%d-%d-%dT%d:%d:%f%d:%dZ", &y, &M, &d, &h, &m,
                    &s, &tzh, &tzm)) {
        if (tzh < 0) {
            tzm = -tzm;
        }
    }

    this->_simStart = cJulian(y, M, d, h, m, s);
    _simStart.addHour(-tzh);
    _simStart.addMin(-tzm);
}

cJulian GlobalJulianDate::currentSimTime() const {
    return simTime2JulianDate(SIMTIME_DBL(omnetpp::simTime()));
}

cJulian GlobalJulianDate::simTime2JulianDate(double simTime) const {
    cJulian tmpJulian(this->_simStart);
    tmpJulian.addSec(simTime);
    return tmpJulian;
}

double GlobalJulianDate::julianDate2SimTime(const cJulian& simTime) const {
    return simTime.spanSec(this->_simStart);
}

} //namespace
