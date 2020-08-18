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

#ifndef EST_GLOBAL_CONFIG_H_
#define EST_GLOBAL_CONFIG_H_

/*
 * Global definitions & configuration are defined here
 */

#include <cmath>
#include <inet/common/Units.h>
#include "estnet/common/ESTNETDefs.h"

namespace estnet {

// namespace shortcuts
namespace inetu = inet::units::values;

// global typedefs
typedef std::tuple<unsigned int, unsigned int> AppAddress;
typedef std::tuple<AppAddress, long> SequenceNumber;

/*
 * Constants definitions
 */

#ifndef PI
#define PI 3.1415926535
#endif // PI

const double TWOPI = 2.0 * PI;
const double RADS_PER_DEG = PI / 180.0;
const double DEGS_PER_RAD = 180.0 / PI;

const double GM = 398601.2; // Earth gravitational constant, km^3/sec^2
const double GEOSYNC_ALT = 42241.892;                      // km
const double EARTH_DIA = 12800.0;                          // km
const double EARTH_AVG_R = 6378137.0;                      // m
const double EARTH_SPIN = 360.0 / 86164.099;               // deg/sec
const double DAY_SIDERAL = (23 * 3600) + (56 * 60) + 4.09; // sec
const double DAY_24HR = (24 * 3600);                       // sec
const double WGS_84_RADIUS_EQUATOR = 6378137.0;            // m
const double WGS_84_RADIUS_POLAR = 6356752.3142;           // m

const double AE = 1.0;
const double AU = 149597870.0; // Astronomical unit (km) (IAU 76)
const double SR = 696000.0;    // Solar radius (km)      (IAU 76)
const double TWOTHRD = 2.0 / 3.0;
const double XKMPER_WGS72 = 6378.135; // Earth equatorial radius - km (WGS '72)
const double F = 1.0 / 298.26;        // Earth flattening (WGS '72)
const double GE = 398600.8;           // Earth gravitational constant (WGS '72)
const double J2 = 1.0826158E-3;       // J2 harmonic (WGS '72)
const double J3 = -2.53881E-6;        // J3 harmonic (WGS '72)
const double J4 = -1.65597E-6;        // J4 harmonic (WGS '72)
const double CK2 = J2 / 2.0;
const double CK4 = -3.0 * J4 / 8.0;
const double XJ3 = J3;
const double E6A = 1.0e-06;
const double QO = AE + 120.0 / XKMPER_WGS72;
const double S = AE + 78.0 / XKMPER_WGS72;
const double HR_PER_DAY = 24.0;           // Hours per day   (solar)
const double MIN_PER_DAY = 1440.0;        // Minutes per day (solar)
const double SEC_PER_DAY = 86400.0;       // Seconds per day (solar)
const double SEC_PER_DAY_SID = 86164.099; // Seconds per day (sidereal)
const double OMEGA_E = 1.00273790934;     // Earth rotation per sideral day
const double K_B = 1.380649E-23;          // Boltzmann Constant

}  // namespace estnet

#endif /* EST_GLOBAL_CONFIG_H_ */
