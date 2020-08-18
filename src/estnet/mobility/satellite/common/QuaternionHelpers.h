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

#ifndef __UTILS_QUATERNION_HELPERS_H__
#define __UTILS_QUATERNION_HELPERS_H__

#include <inet/common/geometry/common/Coord.h>
#include <inet/common/geometry/common/Quaternion.h>

namespace estnet {

void quaternionToMatrix(inet::Quaternion q, double R[3][3]);

double getQuaternionValueAt(inet::Quaternion &q, unsigned int i);

void setQuaternionValueAt(inet::Quaternion &q, unsigned int i, double value);

double getAlignmentAngle(inet::Quaternion orientation, inet::Coord position,
        inet::Coord targetPosition);

}  // namespace estnet

#endif
