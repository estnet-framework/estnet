//
// coord.h
//
// Copyright 2002-2003 Michael F. Henry
//
#pragma once

#include "estnet/global_config.h"

namespace estnet {

//////////////////////////////////////////////////////////////////////
// Geocentric coordinates.class ESTNET_API cCoordGeo
{
public:
    cCoordGeo();
    cCoordGeo(double lat, double lon, double alt) :
            m_Lat(lat), m_Lon(lon), m_Alt(alt)
    {
    }

    double m_Lat; // Latitude,  radians (negative south)
    double m_Lon; // Longitude, radians (negative west)
    double m_Alt; // Altitude,  km      (above mean sea level)
};

//////////////////////////////////////////////////////////////////////
// Topocentric-Horizon coordinates.class ESTNET_API cCoordTopo
{
public:
    cCoordTopo();
    cCoordTopo(double az, double el, double rng, double rate) :
            m_Az(az), m_El(el), m_Range(rng), m_RangeRate(rate)
    {
    }

    double m_Az;        // Azimuth, radians
    double m_El;        // Elevation, radians
    double m_Range;     // Range, kilometers
    double m_RangeRate; // Range rate of change, km/sec
                        // Negative value means "towards observer"
};

}  // namespace estnet
