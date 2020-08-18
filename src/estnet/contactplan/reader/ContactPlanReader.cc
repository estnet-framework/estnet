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

#include "ContactPlanReader.h"

#include <fstream> // ifstream
#include <iostream> // cerr

namespace estnet {

std::vector<contact_plan_entry_t*> ContactPlanReader::read(
        const std::string &contactPlanFilePath) {
    std::vector<contact_plan_entry_t*> contacts;
    if (!contactPlanFilePath.empty()) {
        std::ifstream contactPlanFile;
        contactPlanFile.open(contactPlanFilePath);
        if (!contactPlanFile.is_open()) {
            std::cerr << "Tried to open: " << contactPlanFilePath << std::endl;
            throw std::invalid_argument(
                    "Could not open contact plan file! Make sure it exists, if not run ./createContactPlan.sh");
        }

        int64_t startTime, endTime;
        unsigned int sourceNodeId, sinkNodeId;
        unsigned int maxNodeId = 1;
        unsigned int maxSatNodeId = 1;
        int64_t bitrate, range;
        int64_t maxSimTime;
        std::string lineBuffer;
        while (!contactPlanFile.eof()) {
            // a # (0x23) at the start of the line means we'll skip the whole line
            if (contactPlanFile.peek() == 0x23) {
                std::getline(contactPlanFile, lineBuffer);
                if (lineBuffer.find("# Satellites: 1 - ")
                        != std::string::npos) {
                    sscanf(lineBuffer.c_str(), "# Satellites: 1 - %d",
                            &maxSatNodeId);
                    maxNodeId = maxSatNodeId;
                }
                if (lineBuffer.find("# Ground Stations:")
                        != std::string::npos) {
                    unsigned int tmp;
                    sscanf(lineBuffer.c_str(), "# Ground Stations: %d - %d",
                            &tmp, &maxNodeId);
                }
                if (lineBuffer.find("# sim-time-limit: ")
                        != std::string::npos) {
                    sscanf(lineBuffer.c_str(), "# sim-time-limit: %lld",
                            &maxSimTime);
                }
                continue;
            }
            contactPlanFile >> startTime;
            contactPlanFile >> endTime;
            contactPlanFile >> sourceNodeId;
            contactPlanFile >> sinkNodeId;
            contactPlanFile >> bitrate;
            contactPlanFile >> range;
            // read anything else left in the line and throw away
            std::getline(contactPlanFile, lineBuffer);
            // check if we reached eof, otherwise we recycle the last line
            if (contactPlanFile.eof()) {
                break;
            }
            auto newContact = new contact_plan_entry_t;
            newContact->startTime = startTime;
            newContact->endTime = endTime;
            newContact->sourceNodeId = sourceNodeId;
            newContact->sinkNodeId = sinkNodeId;
            newContact->bitrate = bitrate;
            newContact->range = range;
            newContact->minDistance = -1;
            newContact->maxDistance = -1;
            newContact->sourceIsGroundStation = (sourceNodeId > maxSatNodeId);
            newContact->sinkIsGroundStation = (sinkNodeId > maxSatNodeId);
            newContact->enabled = true;
            newContact->maxSimTime = maxSimTime;

            contacts.push_back(newContact);
        }
    }
    return contacts;
}

}  // namespace estnet
