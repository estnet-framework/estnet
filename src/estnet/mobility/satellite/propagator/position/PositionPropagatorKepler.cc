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

#include "PositionPropagatorKepler.h"

namespace estnet {

Register_Class(PositionPropagatorKepler)

void PositionPropagatorKepler::initialize() {
    /// orbit parameters
    a = par("a");
    e = par("e");
    double i_deg = par("i");
    double raan_deg = par("raan");
    double aop_deg = par("aop");
    double v_deg = par("v");
    i = i_deg * RADS_PER_DEG;
    raan = raan_deg * RADS_PER_DEG;
    aop = aop_deg * RADS_PER_DEG;
    v = v_deg * RADS_PER_DEG;

    // aproximate initial mean anomaly from given initial true anomaly
    M_ini = GetMeanAnomalyFromTrueAnomaly(v);


}

void PositionPropagatorKepler::handleMessage(omnetpp::cMessage *msg) {
}

bool PositionPropagatorKepler::needsStateUpdate(cJulian const &targetTime) {
    // Check if the time is different to the last update
    if(this->_lastUpdateTime.toGMST() == targetTime.toGMST())
    {
        return false;
    }
    
    return true;
}

void PositionPropagatorKepler::propagateState(cJulian const &targetTime,
        state_type &newState) {
    double time = GlobalJulianDate::getInstance().julianDate2SimTime(
            targetTime);

    cEci temp = GetState(time);
    cEci state;
    state.setPos(temp.getPos());
    state.setVel(temp.getVel());
    state.setDate(targetTime);
    state.setUnitsM();

    newState.fromECI(state);
    _lastUpdateTime = targetTime;
    _currentState = newState;
}

double PositionPropagatorKepler::getOrbitalPeriod() const {
    return (2.0 * M_PI * sqrt(pow(a, 3) / GM));
}

double PositionPropagatorKepler::getOrbitalRadius(
        cJulian const &targetTime) const {
    double time = GlobalJulianDate::getInstance().julianDate2SimTime(
            targetTime);
    double v = GetTrueAnomaly(time);
    return GetRadiusFromTrueAnomaly(v);
}

double PositionPropagatorKepler::GetMeanAnomaly(double time) const {
    // calculated average rate of sweep
    double T = this->getOrbitalPeriod();
    double n = 2 * M_PI / T;


    // solve for mean anomaly
    double M = n * (time) + M_ini;
    while(M > 2 * M_PI){
        M -= 2 * M_PI;
    }
    return M;
}

double PositionPropagatorKepler::GetMeanAnomalyFromTrueAnomaly(double v) const {
    // Using series expansion for conversion
    double M = v;
    M += -e * sin(v);
    M += (3/4*pow(e,2) + 1/8*pow(e,4))*sin(2*v);
    M += -1/3*pow(e,3) * sin(3*v);
    M += 5/32*pow(e,4) * sin(4*v);

    return M;
}

/***
 * This method is converted from the JAVA sourcecode of Orekit 7.2
 *\src\main\java\org\orekit\orbits\KeplerianOrbit.java -> 'private double
 *meanToEllipticEccentric(final double M)'
 */
double PositionPropagatorKepler::GetEccentricAnomaly(double time) const {
    // for kepler solution:
    double k1 = 3 * M_PI * 2;
    double k2 = M_PI - 1;
    double k3 = 6 * M_PI - 1;
    double A = 3 * k2 * (k2 / k1);
    double B = k3 * k3 / (6 * k1);

    double MA = this->GetMeanAnomaly(time);    //< get mean anomaly
    double reduceM = this->NormalizeAngle(MA); //< reduce M to [-PI PI) interval


    // compute start value according to A. W. Odell and R. H. Gooding S12 starter
    double E;
    if (fabs(reduceM) < (1.0 / 6.0)) {
        E = reduceM + this->e * (cbrt(6 * reduceM) - reduceM); //< cbrt is cubic root
    } else {
        E = reduceM;
    }

    double e1 = 1 - this->e;
    bool noCancellationRisk = (e1 + E * E / 6) >= 0.1;

    // perform two iterations, each consisting of one Halley step and one
    // Newton-Raphson step
    for (int j = 0; j < 100; ++j) {
        double f;
        double fd;
        double fdd = this->e * sin(E);
        double fddd = this->e * cos(E);
        if (noCancellationRisk) {
            f = (E - fdd) - reduceM;
            fd = 1 - fddd;
        } else {
            f = this->eMeSinE(E) - reduceM;
            double s = sin(0.5 * E);
            fd = e1 + 2 * e * s * s;
        }
        double dee = f * fd / (0.5 * f * fdd - fd * fd);

        // update eccentric anomaly, using expressions that limit underflow problems
        double w = fd + 0.5 * dee * (fdd + dee * fddd / 3);
        fd += dee * (fdd + 0.5 * dee * fddd);

        E -= (f - dee * (fd - w)) / fd;
    }

    // expand the result back to original range
    E += MA - reduceM;

    return E;
}

/***
 * This method is converted from the JAVA sourcecode of Orekit 7.2
 * \src\main\java\org\orekit\orbits\KeplerianOrbit.java
 */
double PositionPropagatorKepler::eMeSinE(double E) const {
    double x = (1 - this->e) * sin(E);
    double mE2 = -E * E;
    double term = E;
    double d = 0;

    // the inequality test below IS intentional and should NOT be replaced by a
    // check with a small tolerance
    double x0;
    while (x != x0) {
        d += 2;
        term *= mE2 / (d * (d + 1));
        x0 = x;
        x = x - term;
    }
    return x;
}

double PositionPropagatorKepler::NormalizeAngle(double x) {
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0) {
        x += 2 * M_PI;
    }
    return x - M_PI;
}

double PositionPropagatorKepler::GetTrueAnomaly(double time) const {
    return (atan(
            sqrt((1 + this->e) / (1 - this->e))
                    * tan(this->GetEccentricAnomaly(time) / 2)) * 2);
}

double PositionPropagatorKepler::GetRadiusFromEccentricAnomaly(
        double EA) const {
    return (this->a * (1 - (this->e * cos(EA))));
}

double PositionPropagatorKepler::GetRadiusFromTrueAnomaly(double TA) const {
    double upper = this->a * (1 - pow(this->e, 2));
    double lower = 1 + (this->e * cos(TA));

    return (upper / lower);
}

double PositionPropagatorKepler::GetSpecificAngularmomentum() const {
    return sqrt(GM * a * (1 - pow(e, 2)));
}

cEci PositionPropagatorKepler::GetState(double time) const {
    cEci ret;
    cVector pos, vel;

    // helper variables
    double v = GetTrueAnomaly(time);
    double r = GetRadiusFromTrueAnomaly(v);
    double h = GetSpecificAngularmomentum();
    double p = a * (1 - pow(e, 2));

    double sinI = sin(i);
    double cosI = cos(i);
    double cosRaan = cos(raan);
    double sinRaan = sin(raan);
    double cosAopTrue = cos(aop + v);
    double sinAopTrue = sin(aop + v);

    // position
    pos.x = r * (cosRaan * cosAopTrue - sinRaan * sinAopTrue * cosI) * 1000;
    pos.y = r * (sinRaan * cosAopTrue + cosRaan * sinAopTrue * cosI) * 1000;
    pos.z = r * (sinAopTrue * sinI) * 1000;

    // velocity
    vel.x = (pos.x * h * e) / (r * p);
    vel.x *= sin(v);
    vel.x -= (h / r) * (cosRaan * sinAopTrue + sinRaan * cosAopTrue * cosI)
            * 1000;

    vel.y = (pos.y * h * e) / (r * p);
    vel.y *= sin(v);
    vel.y -= (h / r) * (sinRaan * sinAopTrue - cosRaan * cosAopTrue * cosI)
            * 1000;

    vel.z = (pos.z * h * e) / (r * p);
    vel.z *= sin(v);
    vel.z += (h / r) * (sinI * cosAopTrue) * 1000;

    ret.setPos(pos);
    ret.setVel(vel);
    ret.setUnitsM();
    ret.setDate(GlobalJulianDate::getInstance().simTime2JulianDate(time));

    return ret;
}

}  // namespace estnet
