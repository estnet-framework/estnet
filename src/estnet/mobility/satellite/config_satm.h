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

#ifndef SATMOBILITY_CONFIG_SATM_H_
#define SATMOBILITY_CONFIG_SATM_H_

#define USE_INET_COORDS       // set the cVector class to inet::Coord class
#define USE_INET_EULER_ANGLES // set the euler angle class to inet::EulerAngles class
#define USE_INET_QUATERNIONS // set the quaternion class to inet::Quaternion.

#include "estnet/global_config.h"

#ifdef USE_INET_COORDS
#include "inet/common/geometry/common/Coord.h"
#define cVector inet::Coord
#endif

#endif /* SATMOBILITY_CONFIG_SATM_H_ */
