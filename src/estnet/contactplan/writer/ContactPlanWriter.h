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

#ifndef __CONTACTPLANS_WRITER_H__
#define __CONTACTPLANS_WRITER_H__

#include <vector>
#include <string>

#include "estnet/contactplan/common/Contacts.h"

namespace estnet {

/**
 * Writes contact plans to files.
 */
class ESTNET_API ContactPlanWriter {
public:
    /** @brief writes the given contact plan to the given file */
    void write(const std::vector<contact_plan_entry_t*> &contactPlan,
            const std::string &filename, int64_t maxSimTime);
};

}  // namespace estnet

#endif
