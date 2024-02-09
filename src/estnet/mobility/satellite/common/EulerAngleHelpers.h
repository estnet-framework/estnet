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

#ifndef __UTILS_EULER_ANGLE_HELPERS_H__
#define __UTILS_EULER_ANGLE_HELPERS_H__

#include <inet/common/geometry/common/Coord.h>
#include <inet/common/geometry/common/EulerAngles.h>

#include "estnet/common/matrix/Matrix.h"


namespace estnet {

/** @brief Converts the given rotation matrix R
 *  into yaw, pitch, roll euler angles.
 *  Rotation order is HPR/YPR/ZXY.
 */
void getEulerAnglesFromMatrix(M4x4d& R, double &yaw, double &pitch,
        double &roll);

/** @brief Converts the given yaw, pitch, roll euler angles
 *  into a rotation matrix.
 *  Rotation order is HPR/YPR/ZXY.
 */
void getMatrixFromEulerAngles(double yaw, double pitch, double roll,
        M4x4d& matrix);

/** @brief Combines two euler angle sets together.
 *  Rotation order is HPR/YPR/ZXY.
 */
inet::EulerAngles combineEulerAngles(const inet::EulerAngles &rotEul1,
        const inet::EulerAngles &rotEul2);

/** @brief Calculates the line-of-sight angles from
 *  posA (world coordinate frame) with orientation
 *  attA (rotation into local coordinate frame)
 *  to posB (world coordinate frame).
 *  Rotation order is HPR/YPR/ZXY.
 */
inet::EulerAngles angleBetweenPositions(const inet::Coord &posA,
        const inet::EulerAngles &attA, const inet::Coord &posB);

double getEulerAngleValueAt(inet::EulerAngles &angle, int i);
void setEulerAngleValueAt(inet::EulerAngles &angle, int i, double value);

}  // namespace estnet

#endif
