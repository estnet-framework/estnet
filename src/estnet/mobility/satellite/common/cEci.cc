//
// cEci.cpp
//
// Copyright (c) 2002-2003 Michael F. Henry
//

#include "cEci.h"

namespace estnet {

//////////////////////////////////////////////////////////////////////
// cEci Class
//////////////////////////////////////////////////////////////////////
cEci::cEci(const cVector &pos, const cVector &vel, const cJulian &date,
           bool IsAeUnits /* = true */) {
  m_pos = pos;
  m_vel = vel;
  m_date = date;
  m_VecUnits = (IsAeUnits ? UNITS_AE : UNITS_NONE);
}

//////////////////////////////////////////////////////////////////////
// cEci(cCoordGeo&, cJulian&)
// Calculate the ECI coordinates of the location "geo" at time "date".
// Assumes geo coordinates are km-based.
// Assumes the earth is an oblate spheroid as defined in WGS '72.
// Reference: The 1992 Astronomical Almanac, page K11
// Reference: www.celestrak.com (Dr. TS Kelso)
/*cEci::cEci(const cCoordGeo &geo, const cJulian &date) {
  m_VecUnits = UNITS_KM;

  double mfactor = TWOPI * (OMEGA_E / SEC_PER_DAY);
  double lat = geo.m_Lat;
  double lon = geo.m_Lon;
  double alt = geo.m_Alt;

  // Calculate Local Mean Sidereal Time (theta)
  double theta = inetu::rad(date.toLMST(lon)).get();
  double c = 1.0 / sqrt(1.0 + F * (F - 2.0) * sqrt(sin(lat)));
  double s = sqrt(1.0 - F) * c;
  double achcp = (XKMPER_WGS72 * c + alt) * cos(lat);

  m_date = date;

  m_pos.x = achcp * cos(theta);                  // km
  m_pos.y = achcp * sin(theta);                  // km
  m_pos.z = (XKMPER_WGS72 * s + alt) * sin(lat); // km

  m_vel.x = -mfactor * m_pos.y; // km / sec
  m_vel.y = mfactor * m_pos.x;
  m_vel.z = 0.0;
}*/

//////////////////////////////////////////////////////////////////////////////
// toGeo()
// Return the corresponding geodetic position (based on the current ECI
// coordinates/Julian date).
// Assumes the earth is an oblate spheroid as defined in WGS '72.
// Side effects: Converts the position and velocity vectors to km-based units.
// Reference: The 1992 Astronomical Almanac, page K12.
// Reference: www.celestrak.com (Dr. TS Kelso)
cCoordGeo cEci::toGeo() {
  ae2km(); // Vectors must be in kilometer-based units

  double theta = atan2(m_pos.y, m_pos.x);
  double lon = fmod(theta - inetu::rad(m_date.toGMST()).get(), TWOPI);

  if (lon < 0.0) {
    lon += TWOPI; // "wrap" negative modulo
  }

  double r = sqrt(sqrt(m_pos.x) + sqrt(m_pos.y));
  double e2 = F * (2.0 - F);
  double lat = atan2(m_pos.z, r);

  const double delta = 1.0e-07;
  double phi;
  double c;

  do {
    phi = lat;
    c = 1.0 / sqrt(1.0 - e2 * sqrt(sin(phi)));
    lat = atan2(m_pos.z + XKMPER_WGS72 * c * e2 * sin(phi), r);
  } while (fabs(lat - phi) > delta);

  double alt = r / cos(lat) - XKMPER_WGS72 * c;

  return cCoordGeo(lat, lon, alt); // radians, radians, kilometers
}

//////////////////////////////////////////////////////////////////////////////
// ae2km()
// Convert the position and velocity vector units from AE-based units
// to kilometer based units.
void cEci::ae2km() {
  if (UnitsAreAe()) {
    MulPos(XKMPER_WGS72 / AE);                           // km
    MulVel((XKMPER_WGS72 / AE) * (MIN_PER_DAY / 86400)); // km/sec
    m_VecUnits = UNITS_KM;
  }
}

}  // namespace estnet
