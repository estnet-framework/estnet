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

#include "QuaternionHelpers.h"

namespace estnet {

void quaternionToMatrix(inet::Quaternion q, double R[3][3]) {
    // column1
    R[0][0] = 2 * q.s * q.s - 1 + 2 * q.v.x * q.v.x;
    R[1][0] = 2 * q.v.y * q.v.x + 2 * q.s * q.v.z;
    R[2][0] = 2 * q.v.z * q.v.x - 2 * q.s * q.v.y;

    // column 2
    R[0][1] = 2 * q.v.y * q.v.x - 2 * q.s * q.v.z;
    R[1][1] = 2 * q.s * q.s - 1 + 2 * q.v.y * q.v.y;
    R[2][1] = 2 * q.v.y * q.v.z + 2 * q.s * q.v.x;

    // column 3
    R[0][2] = 2 * q.v.z * q.v.x + 2 * q.s * q.v.y;
    R[1][2] = 2 * q.v.y * q.v.z - 2 * q.s * q.v.x;
    R[2][2] = 2 * q.s * q.s - 1 + 2 * q.v.z * q.v.z;
}

double getQuaternionValueAt(inet::Quaternion &q, unsigned int i) {
    switch (i) {
    case 0:
        return q.getS();
        break;
    case 1:
        return q.getV().getX();
        break;
    case 2:
        return q.getV().getY();
        break;
    case 3:
        return q.getV().getZ();
        break;
    default:
        throw omnetpp::cRuntimeError(
                "Quaterions only have 4 values index from 0 to 3 and not %d",
                i);
        break;
    }
}

void setQuaternionValueAt(inet::Quaternion &q, unsigned int i, double value) {
    switch (i) {
    case 0:
        q.setS(value);
        break;
    case 1:
        q.setV(inet::Coord(value, q.getV().getY(), q.getV().getZ()));
        break;
    case 2:
        q.setV(inet::Coord(q.getV().getX(), value, q.getV().getZ()));
        break;
    case 3:
        q.setV(inet::Coord(q.getV().getX(), q.getV().getY(), value));
        break;
    default:
        throw omnetpp::cRuntimeError(
                "Quaterions only have 4 values index from 0 to 3 and not %d",
                i);
        break;
    }
}

double getAlignmentAngle(inet::Quaternion orientation, inet::Coord position,
        inet::Coord targetPosition) {

    inet::Coord connectingVector = targetPosition - position;
    connectingVector.normalize();

    double angle = acos(
            orientation.rotate(inet::Coord::X_AXIS) * connectingVector);

    return angle;
}

}  // namespace estnet
