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
#include "estnet/common/matrix/Matrix.h"
#include <cmath>

namespace estnet {

void getEulerAnglesFromMatrix(double R[3][3], double &yaw, double &pitch,
        double &roll) {
    // figure out the euler angles that lead to this rotation matrix
    // HPR/YPR/ZXY rotation order leads to the following rotation matrix:
    //  Ch * Cr - Sh * Sp * Sr     -Sh * Cp     Ch * Sr + Sh * Sp * Cr
    //  Sh * Cr + Ch * Sp * Sr      Ch * Cp     Sh * Sr - Ch * Sp * Cr
    // -Cp * Sr                     Sp          Cp * Cr
    // https://www.geometrictools.com/Documentation/EulerAngles.pdf

    /*for (int i = 0; i < 3; i++) {
     for(int j =0; j < 3; j++) {
     if (fabs(R[i][j]) < 1e-3) {
     R[i][j] = 0;
     }
     }
     }*/

    //if (std::fabs(R[2][1]) < 0.99999) {
    if (R[2][1] < 1) {
        if (R[2][1] > -1) {
            yaw = atan2(-R[0][1], R[1][1]);
            pitch = asin(R[2][1]);
            roll = atan2(-R[2][0], R[2][2]);
        } else {
            yaw = -atan2(R[0][2], R[0][0]);
            pitch = -M_PI / 2.0;
            roll = 0;
        }
    } else {
        yaw = atan2(R[0][2], R[0][0]);
        pitch = M_PI / 2.0;
        roll = 0;
    }
    /*} else {
     yaw   =  atan2( R[1][0], R[0][0]);
     pitch =  asin(  R[2][1]);
     roll  =  0;
     }*/
}

void getMatrixFromEulerAngles(double yaw, double pitch, double roll,
        double matrix[3][3]) {
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

    /*for (int i = 0; i < 3; i++) {
     for(int j =0; j < 3; j++) {
     if (fabs(rotationH(i, j)) < 1e-3) {
     rotationH(i, j) = 0;
     }
     }
     }
     for (int i = 0; i < 3; i++) {
     for(int j =0; j < 3; j++) {
     if (fabs(rotationP(i, j)) < 1e-3) {
     rotationP(i, j) = 0;
     }
     }
     }
     for (int i = 0; i < 3; i++) {
     for(int j =0; j < 3; j++) {
     if (fabs(rotationR(i, j)) < 1e-3) {
     rotationR(i, j) = 0;
     }
     }
     }*/

    // using osg conventions for rotation order and axises
    M4x4d combined = rotationH * rotationP * rotationR;

    /*for (int i = 0; i < 3; i++) {
     for(int j =0; j < 3; j++) {
     if (fabs(combined(i, j)) < 1e-3) {
     combined(i, j) = 0;
     }
     }
     }*/

    /*EV_TRACE << "rotationH " << yaw << omnetpp::endl;
     EV_TRACE << rotationH << omnetpp::endl << omnetpp::endl;
     EV_TRACE << "rotationP " << pitch << omnetpp::endl;
     EV_TRACE << rotationP << omnetpp::endl << omnetpp::endl;
     EV_TRACE << "rotationR " << roll << omnetpp::endl;
     EV_TRACE << rotationR << omnetpp::endl << omnetpp::endl;
     EV_TRACE << "rotation" << omnetpp::endl;
     EV_TRACE << combined << omnetpp::endl << omnetpp::endl;*/

    matrix[0][0] = combined(0, 0);
    matrix[0][1] = combined(0, 1);
    matrix[0][2] = combined(0, 2);
    matrix[1][0] = combined(1, 0);
    matrix[1][1] = combined(1, 1);
    matrix[1][2] = combined(1, 2);
    matrix[2][0] = combined(2, 0);
    matrix[2][1] = combined(2, 1);
    matrix[2][2] = combined(2, 2);
}

inet::EulerAngles combineEulerAngles(const inet::EulerAngles &rotEul1,
        const inet::EulerAngles &rotEul2) {
    double rotMatPtr1[3][3], rotMatPtr2[3][3];
    getMatrixFromEulerAngles(rotEul1.alpha.get(), rotEul1.beta.get(),
            rotEul1.gamma.get(), rotMatPtr1);
    getMatrixFromEulerAngles(rotEul2.alpha.get(), rotEul2.beta.get(),
            rotEul2.gamma.get(), rotMatPtr2);

    M4x4d rotMat1;
    rotMat1 << rotMatPtr1[0][0], rotMatPtr1[0][1], rotMatPtr1[0][2], 0.0, rotMatPtr1[1][0], rotMatPtr1[1][1], rotMatPtr1[1][2], 0.0, rotMatPtr1[2][0], rotMatPtr1[2][1], rotMatPtr1[2][2], 0.0, 0.0, 0.0, 0.0, 1.0;
    M4x4d rotMat2;
    rotMat2 << rotMatPtr2[0][0], rotMatPtr2[0][1], rotMatPtr2[0][2], 0.0, rotMatPtr2[1][0], rotMatPtr2[1][1], rotMatPtr2[1][2], 0.0, rotMatPtr2[2][0], rotMatPtr2[2][1], rotMatPtr2[2][2], 0.0, 0.0, 0.0, 0.0, 1.0;

    double combinedMatPtr[3][3];
    M4x4d combined = rotMat1 * rotMat2;
    combinedMatPtr[0][0] = combined(0, 0);
    combinedMatPtr[0][1] = combined(0, 1);
    combinedMatPtr[0][2] = combined(0, 2);
    combinedMatPtr[1][0] = combined(1, 0);
    combinedMatPtr[1][1] = combined(1, 1);
    combinedMatPtr[1][2] = combined(1, 2);
    combinedMatPtr[2][0] = combined(2, 0);
    combinedMatPtr[2][1] = combined(2, 1);
    combinedMatPtr[2][2] = combined(2, 2);
    //EV_TRACE << "combined rotation matrix" << endl;
    //EV_TRACE << combined << endl << endl;

    inet::EulerAngles result;
    getEulerAnglesFromMatrix(combinedMatPtr,
            const_cast<double&>(result.alpha.get()),
            const_cast<double&>(result.beta.get()),
            const_cast<double&>(result.gamma.get()));
    return result;
}

inet::EulerAngles angleBetweenPositions(const inet::Coord &posA,
        const inet::EulerAngles &attA, const inet::Coord &posB) {
    Eigen::Vector3d ourPositionECI_e(posA.x, posA.y, posA.z);
    Eigen::Vector3d targetPositionECI_e(posB.x, posB.y, posB.z);
    double rotMatPtr1[3][3];
    getMatrixFromEulerAngles(attA.alpha.get(), attA.beta.get(),
            attA.gamma.get(), rotMatPtr1);
    M3x3d ourOrientation_e;
    ourOrientation_e << rotMatPtr1[0][0], rotMatPtr1[0][1], rotMatPtr1[0][2], rotMatPtr1[1][0], rotMatPtr1[1][1], rotMatPtr1[1][2], rotMatPtr1[2][0], rotMatPtr1[2][1], rotMatPtr1[2][2];

    Eigen::Vector3d transmissionDirection_e = targetPositionECI_e
            - ourPositionECI_e;
    transmissionDirection_e.normalize();
    Eigen::Vector3d transmissionDirectionLocal_e = ourOrientation_e.transpose()
            * transmissionDirection_e;
    /*EV_TRACE << "angleBetweenPositions posA " << posA << endl;
     EV_TRACE << "angleBetweenPositions posB " << posB << endl;
     EV_TRACE << "angleBetweenPositions diff vector " << inet::Coord(transmissionDirection_e(0), transmissionDirection_e(1), transmissionDirection_e(2)) << endl;
     EV_TRACE << "angleBetweenPositions rotated vector " << inet::Coord(transmissionDirectionLocal_e(0), transmissionDirectionLocal_e(1), transmissionDirectionLocal_e(2)) << endl;
     EV_TRACE << "posA" << endl;
     EV_TRACE << ourPositionECI_e << endl << endl;
     EV_TRACE << "posB" << endl;
     EV_TRACE << targetPositionECI_e << endl << endl;
     EV_TRACE << "orientation matrix" << endl;
     EV_TRACE << ourOrientation_e << endl << endl;
     EV_TRACE << "vector" << endl;
     EV_TRACE << transmissionDirection_e << endl << endl;/
     EV_TRACE << "result" << endl;
     EV_TRACE << transmissionDirectionLocal_e << endl << endl;*/

    // if there are really small errors, we set them to zero, as atan2 tends
    // to start rotating 180 deg otherwise
    /*if (std::fabs(transmissionDirectionLocal_e(0)) < 1e-3) {
     transmissionDirectionLocal_e(0) = 0;
     }
     if (std::fabs(transmissionDirectionLocal_e(1)) < 1e-3) {
     transmissionDirectionLocal_e(1) = 0;
     }
     if (std::fabs(transmissionDirectionLocal_e(2)) < 1e-3) {
     transmissionDirectionLocal_e(2) = 0;
     }*/
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
