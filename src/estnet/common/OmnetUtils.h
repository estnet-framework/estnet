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

#ifndef ESTNET_COMMON_OMNETUTILS_H_
#define ESTNET_COMMON_OMNETUTILS_H_

#include "estnet/global_config.h"

namespace estnet {

/**
 * parsing ini parameter of simtime-limit with correct units
 * @param ini parameter of simtime-limit as string
 * @return simulation time limit
 */
inetu::s ESTNET_API parseSimTimeLimit(const char* simTimeLimitString);

}

#endif /* ESTNET_COMMON_OMNETUTILS_H_ */
