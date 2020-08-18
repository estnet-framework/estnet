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

#include "PositionPropagatorSGP4File.h"

#include <fstream>

namespace estnet {

Register_Class(PositionPropagatorSGP4File);

void PositionPropagatorSGP4File::getTleLines(std::string &firstLine,
        std::string &secondLine) {
    int nodeNo =
            this->getParentModule()->getParentModule()->par("nodeNo").intValue()
                    - 1;
    std::string tleFileName = this->par("tleFile").stringValue();
    std::ifstream tleFile;
    tleFile.open(tleFileName);
    if (!tleFile.is_open()) {
        throw omnetpp::cRuntimeError("Could not open TLE file!");
    }
    std::string line;
    unsigned int lineNo = 0;
    unsigned int tleStartLineNo = nodeNo * 3;
    unsigned int tleEndLineNo = (nodeNo + 1) * 3;
    while (getline(tleFile, line)) {
        if (lineNo == tleStartLineNo + 1) {
            firstLine = line;
        } else if (lineNo == tleStartLineNo + 2) {
            secondLine = line;
        } else if (lineNo >= tleEndLineNo) {
            break;
        }
        lineNo++;
    }
}

}  // namespace estnet
