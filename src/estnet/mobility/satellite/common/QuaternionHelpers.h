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

#include "estnet/common/matrix/Matrix.h"


namespace estnet {

/**
 * Converts a quaternion into a rotation matrix
 * @param q: quaternion to be converted
 * @param R: Array, where the rotation matrix is written to
 */
void quaternionToMatrix(inet::Quaternion q, double R[3][3]);

/**
 * Converts a quaternion into a rotation matrix
 * @param q: quaternion to be converted
 * @param R: rotation matrix reference, where the result is written to
 */
void quaternionToMatrix(inet::Quaternion q, M4x4d& R);

/**
 * Access an element of the quaternion at the given index
 * Throws an exception if the index is not valid (0-3)
 * @param q: reference to the quaternion that should be accessed
 * @param i: index for access (0  is scalar value, 1-3 the vector elements)
 * @return: element of passed quaternion at given index
 */
double getQuaternionValueAt(inet::Quaternion &q, unsigned int i);

/**
 * Sets an element of the quaternion at the given index to a new value
 * Throws an exception if the index is not valid (0-3)
 * @param q: reference to the quaternion that should be changed
 * @param i: index which should be set (0  is scalar value, 1-3 the vector elements)
 * @param value: value that is set to the given index
 */
void setQuaternionValueAt(inet::Quaternion &q, unsigned int i, double value);


/**
 * Calculate the alignment angle from a given position and orientation to a target coordinate
 * The pointing axis is the x-axis.
 * @param orientation: Current orientation of the node with reference to a certain (global) frame
 * @param position: Current position of the node in same reference frame
 * @param targetPosition: Current position of the target in same reference frame
 */
double getAlignmentAngle(inet::Quaternion orientation, inet::Coord position,
        inet::Coord targetPosition);

}  // namespace estnet

#endif
