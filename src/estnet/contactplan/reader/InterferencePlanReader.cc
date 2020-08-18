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

#include "InterferencePlanReader.h"

#include <fstream> // ifstream

namespace estnet {

InterferencePlan InterferencePlanReader::read(
        const std::string &interferencePlanFilePath) {
    InterferencePlan interferences;
    if (!interferencePlanFilePath.empty()) {
        std::ifstream interferencePlanFile;
        interferencePlanFile.open(interferencePlanFilePath);
        if (!interferencePlanFile.is_open()) {
            throw std::invalid_argument(
                    "Could not open interference plan file!");
        }

        int64_t startTime, endTime;
        unsigned int sourceNodeId, sinkNodeId, interferingNodeId;
        std::string lineBuffer;
        while (!interferencePlanFile.eof()) {
            // a # (0x23) at the start of the line means we'll skip the whole line
            if (interferencePlanFile.peek() == 0x23) {
                std::getline(interferencePlanFile, lineBuffer);
                continue;
            }
            std::vector<unsigned int> interferingNodeIds;

            interferencePlanFile >> startTime;
            interferencePlanFile >> endTime;
            interferencePlanFile >> interferingNodeId;
            interferingNodeIds.push_back(interferingNodeId);
            while (interferencePlanFile.peek() == ',') {
                char dummy;
                interferencePlanFile >> dummy;
                interferencePlanFile >> interferingNodeId;
                interferingNodeIds.push_back(interferingNodeId);
            }
            interferencePlanFile >> sourceNodeId;
            interferencePlanFile >> sinkNodeId;
            // read anything else left in the line and throw away
            std::getline(interferencePlanFile, lineBuffer);
            // check if we reached eof, otherwise we recycle the last line
            if (interferencePlanFile.eof()) {
                break;
            }
            auto newInterference = new interference_plan_entry_t;
            newInterference->startTime = startTime;
            newInterference->endTime = endTime;
            newInterference->interferingNodeId = interferingNodeIds;
            newInterference->sourceNodeId = sourceNodeId;
            newInterference->sinkNodeId = sinkNodeId;
            newInterference->enabled = true;

            interferences.push_back(newInterference);
        }
    }
    return interferences;
}

}  // namespace estnet
