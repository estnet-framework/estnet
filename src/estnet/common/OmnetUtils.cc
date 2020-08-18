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
// @author: Timon Petermann
//

#include "OmnetUtils.h"

namespace estnet {

inetu::s parseSimTimeLimit(const char* simTimeLimitString) {
    double value;
    char unit1, unit2, unit3;
    int fb = sscanf(simTimeLimitString, "%lf%c%c%c", &value, &unit1, &unit2,
            &unit3);

    // check unit
    if (fb == 3 && unit2 == 's') {
        if (unit1 == 'm') {
            value *= 1e-3;
        } else if (unit1 == 'u') {
            value *= 1e-6;
        } else if (unit1 == 'n') {
            value *= 1e-9;
        } else if (unit1 == 'p') {
            value *= 1e-12;
        }
    } else if (fb == 4) {
        if (unit1 == 'm' && unit2 == 'i' && unit3 == 'n') {
            value *= 60;
        } else {
            throw omnetpp::cRuntimeError(
                    "Coding error: wrong argument in sim-time-limit");
        }
    } else {
        if (unit1 == 'd') {
            value *= 86400;
        } else if (unit1 == 'h') {
            value *= 3600;
        } else if (unit1 != 's') {
            throw omnetpp::cRuntimeError(
                    "Coding error: wrong argument in sim-time-limit");
        }
    }

    return inetu::s(value);
}

}
