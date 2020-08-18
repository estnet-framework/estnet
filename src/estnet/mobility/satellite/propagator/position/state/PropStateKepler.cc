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

#include "PropStateKepler.h"
#include "estnet/global_config.h"

namespace estnet {

void PropStateKepler::asECI(cEci &eci) const {
    // CALCULATIONS INCORRECT, CHECK BEFORE USE
    // Calculate radius from true anomaly
    double upper = _KeplerP._a * (1 - pow(_KeplerP._e, 2));
    double lower = 1 + (_KeplerP._e * cos(_KeplerP._v));
    double r = (upper / lower);

    // Helper variabels
    double h = sqrt(GM * _KeplerP._a * (1 - pow(_KeplerP._e, 2))); // specific angular momentum
    double p = _KeplerP._a * (1 - pow(_KeplerP._e, 2));
    double cosI = cos(_KeplerP._i);
    double cosRaan = cos(_KeplerP._W);
    double sinRaan = sin(_KeplerP._W);
    double cosAopTrue = cos(_KeplerP._w + _KeplerP._v);
    double sinAopTrue = sin(_KeplerP._w + _KeplerP._v);
    double sinI = sin(_KeplerP._i);

    // calcualte position
    cVector pos;
    pos.x = r * (cosRaan * cosAopTrue - sinRaan * sinAopTrue * cosI) * 1000;
    pos.y = r * (sinRaan * cosAopTrue + cosRaan * sinAopTrue * cosI) * 1000;
    pos.z = r * (sinAopTrue * sinI) * 1000;

    eci.setPos(pos); // set position state

    // calculate velocity
    cVector vel;
    vel.x = (pos.x * h * _KeplerP._e) / (r * p);
    vel.x *= sin(_KeplerP._v);
    vel.x -= (h / r) * (cosRaan * sinAopTrue + sinRaan * cosAopTrue * cosI)
            * 1000;

    vel.y = (pos.y * h * _KeplerP._e) / (r * p);
    vel.y *= sin(_KeplerP._v);
    vel.y -= (h / r) * (sinRaan * sinAopTrue - cosRaan * cosAopTrue * cosI)
            * 1000;

    vel.z = (pos.z * h * _KeplerP._e) / (r * p);
    vel.z *= sin(_KeplerP._v);
    vel.z += (h / r) * (sinI * cosAopTrue) * 1000;

    eci.setVel(vel); // set velocity state

    eci.UnitsAreKm(); // set unit to KM
}

void PropStateKepler::fromECI(cEci const &eci) {
    cVector ecc;                // eccentricity vector
    cVector h;                  // specific relative angular momentum
    cVector pos = eci.getPos(); // position vector in eci coordinates
    cVector vel = eci.getVel(); // velocity vector in eci coordinates

    double r = pos.length();
    double absv = vel.length();
    double E = (pow(absv, 2) / 2) - (GM / r);

    h = pos % vel; // Cross Product

    _KeplerP._a = -GM / (2 * E); // a - semimajor axis
    _KeplerP._e = sqrt(1 - pow(h.length(), 2) / (_KeplerP._a * GM)); // e - eccentricity
    _KeplerP._i = acos(h.z / h.length());                  // i - inclination

            // W - RAAN
    cVector n = cVector(-h.y, h.x, 0.0);
    double W = acos(n.x / n.length());
    if (n.y < 0) {
        W = 2 * M_PI - W;
    }

    _KeplerP._W = W;

    ecc = (pos * (((vel * vel / GM) - 1 / pos.length())))
            - vel * ((pos * vel) / GM);

    _KeplerP._w = acos((n * ecc) / (n.length() * ecc.length())); // Aop

    // v - true anomaly

    double v = acos(ecc * pos / (ecc.length() * pos.length()));

    if (pos * vel < 0.0) {
        v = 2 * M_PI - v;
    }

    _KeplerP._v = v;

    _TimeStamp = eci.getDate();
}

}  // namespace estnet
