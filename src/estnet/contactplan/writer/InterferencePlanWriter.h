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

#ifndef __CONTACTPLANS_INTERFERENCE_WRITER_H__
#define __CONTACTPLANS_INTERFERENCE_WRITER_H__

#include <vector>

#include "estnet/contactplan/common/Contacts.h"

namespace estnet {

/**
 * Writes interference plans to files.
 */
class ESTNET_API InterferencePlanWriter {
public:
    /** @brief writes the given interference plan to the given file */
    void write(const std::vector<interference_plan_entry_t*> &interferencePlan,
            const std::string &filename);
    /** @brief writes vector to string, seperated by comma */
    template<typename T>
    std::string writeVector(std::vector<T> vec);

};

}  // namespace estnet

#endif
