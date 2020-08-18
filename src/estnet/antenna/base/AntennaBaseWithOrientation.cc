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

#include "AntennaBaseWithOrientation.h"

#include "AntennaMobility.h"

namespace estnet {

void AntennaBaseWithOrientation::initialize(int stage) {
    AntennaBase::initialize(stage);
    if (stage == 0) {
        // overwriting AntennaBase mobility
        this->mobility = new AntennaMobility(this->mobility,
                inet::units::values::deg(par("yaw")).get(),
                inet::units::values::deg(par("pitch")).get(),
                inet::units::values::deg(par("roll")).get());
    }
}

}  // namespace estnet
