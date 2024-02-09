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

#include "EulerAngleHelpers.h"

#include <cmath>

namespace estnet {

void getEulerAnglesFromMatrix(M4x4d& R, double &yaw, double &pitch,
        double &roll) {
    // figure out the euler angles that lead to this rotation matrix
    // HPR/YPR/ZXY rotation order leads to the following rotation matrix:
    //  Ch * Cr - Sh * Sp * Sr     -Sh * Cp     Ch * Sr + Sh * Sp * Cr
    //  Sh * Cr + Ch * Sp * Sr      Ch * Cp     Sh * Sr - Ch * Sp * Cr
    // -Cp * Sr                     Sp          Cp * Cr
    // https://www.geometrictools.com/Documentation/EulerAngles.pdf

    if (R(2,1) < 1) {
        if (R(2,1) > -1) {
            yaw = atan2(-R(0,1), R(1,1));
            pitch = asin(R(2,1));
            roll = atan2(-R(2,0), R(2,2));
        } else {
            yaw = -atan2(R(0,2), R(0,0));
            pitch = -M_PI / 2.0;
            roll = 0;
        }
    } else {
        yaw = atan2(R(0,2), R(0,0));
        pitch = M_PI / 2.0;
        roll = 0;
    }
}


void getMatrixFromEulerAngles(double yaw, double pitch, double roll,
        M4x4d& matrix) {
    // around z-axis (yaw/heading)
    //  Ch  -Sh    0
    //  Sh   Ch    0
    //   0    0    1
    double h = yaw;
    double ch = cos(h), sh = sin(h);
    M4x4d rotationH = M4x4d::Identity();
    rotationH(0, 0) = ch;
    rotationH(0, 1) = -sh;
    rotationH(1, 0) = sh;
    rotationH(1, 1) = ch;
    // around x-axis (pitch)
    //   1    0    0
    //   0   Cp  -Sp
    //   0   Sp   Cp
    double p = pitch;
    double cp = cos(p), sp = sin(p);
    M4x4d rotationP = M4x4d::Identity();
    rotationP(1, 1) = cp;
    rotationP(1, 2) = -sp;
    rotationP(2, 1) = sp;
    rotationP(2, 2) = cp;
    // around y-axis (roll)
    //  Cr    0   Sr
    //   0    1    0
    // -Sr    0   Cr
    double r = roll;
    double cr = cos(r), sr = sin(r);
    M4x4d rotationR = M4x4d::Identity();
    rotationR(0, 0) = cr;
    rotationR(0, 2) = sr;
    rotationR(2, 0) = -sr;
    rotationR(2, 2) = cr;

    // using osg conventions for rotation order and axises
    matrix = rotationH * rotationP * rotationR;

}

inet::EulerAngles combineEulerAngles(const inet::EulerAngles &rotEul1,
        const inet::EulerAngles &rotEul2) {
    M4x4d rotMat1, rotMat2;
    getMatrixFromEulerAngles(rotEul1.alpha.get(), rotEul1.beta.get(),
            rotEul1.gamma.get(), rotMat1);
    getMatrixFromEulerAngles(rotEul2.alpha.get(), rotEul2.beta.get(),
            rotEul2.gamma.get(), rotMat2);

    M4x4d combined = rotMat1 * rotMat2;

    inet::EulerAngles result;
    getEulerAnglesFromMatrix(combined,
            const_cast<double&>(result.alpha.get()),
            const_cast<double&>(result.beta.get()),
            const_cast<double&>(result.gamma.get()));
    return result;
}

inet::EulerAngles angleBetweenPositions(const inet::Coord &posA,
        const inet::EulerAngles &attA, const inet::Coord &posB) {
    Eigen::Vector3d ourPositionECI_e(posA.x, posA.y, posA.z);
    Eigen::Vector3d targetPositionECI_e(posB.x, posB.y, posB.z);
    M4x4d ourOrientation_e;
    getMatrixFromEulerAngles(attA.alpha.get(), attA.beta.get(),
            attA.gamma.get(), ourOrientation_e);

    Eigen::Vector3d transmissionDirection_e = targetPositionECI_e
            - ourPositionECI_e;
    transmissionDirection_e.normalize();
    Eigen::Vector4d transmissionDirectionLocal_e = ourOrientation_e.transpose()
            * transmissionDirection_e.homogeneous();

    // around z-axis
    double yawErr = -atan2(transmissionDirectionLocal_e(1),
            transmissionDirectionLocal_e(0));
    // around x-axis
    double pitchErr = 0; // is just a rotation around the own antenna axis
    // around y-axis
    double rollErr = atan2(transmissionDirectionLocal_e(2),
            sqrt(
                    pow(transmissionDirectionLocal_e(0), 2)
                            + pow(transmissionDirectionLocal_e(1), 2)));

    return inet::EulerAngles((inet::units::values::rad) yawErr,
            (inet::units::values::rad) pitchErr,
            (inet::units::values::rad) rollErr);
}

double getEulerAngleValueAt(inet::EulerAngles &angle, int i) {
    switch (i) {
    case 0:
        return angle.alpha.get();
    case 1:
        return angle.beta.get();
    case 2:
        return angle.gamma.get();
    }
    throw omnetpp::cRuntimeError(
            "Asked for an euler angle indes that does not exist (valid: 0 - 2)");
}

void setEulerAngleValueAt(inet::EulerAngles &angle, int i, double value) {
    switch (i) {
    case 0:
        angle.alpha = inet::units::values::rad(value);
        break;
    case 1:
        angle.beta = inet::units::values::rad(value);
        break;
    case 2:
        angle.gamma = inet::units::values::rad(value);
        break;
    }
}

}  // namespace estnet
