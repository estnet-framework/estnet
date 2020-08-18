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

#ifndef __CONTACTPLANS_READER_H__
#define __CONTACTPLANS_READER_H__

#include <vector>
#include <string> // string

#include "estnet/contactplan/common/Contacts.h"

namespace estnet {

/**
 * Reads contacts plans from files.
 */
class ESTNET_API ContactPlanReader {
public:
    /** @brief reads the contact plan from the given file */
    std::vector<contact_plan_entry_t*> read(
            const std::string &contactPlanFilePath);
};

}  // namespace estnet

#endif
