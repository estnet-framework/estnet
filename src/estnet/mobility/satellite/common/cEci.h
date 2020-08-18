//
// cEci.h
//
// Copyright (c) 2003 Michael F. Henry
//
#pragma once

#include "estnet/mobility/satellite/config_satm.h"
#include "ccoord.h"
#include <math.h>
#include "estnet/common/time/cJulian.h"

namespace estnet {

//////////////////////////////////////////////////////////////////////
// class cEci
// Encapsulates an Earth-Centered Inertial position, velocity, and time.class ESTNET_API cEci {
public:
  cEci() { m_VecUnits = UNITS_NONE; }
  //cEci(const cCoordGeo &geo, const cJulian &date);
  cEci(const cVector &pos, const cVector &vel, const cJulian &date,
       bool IsAeUnits = true);

  cCoordGeo toGeo();

  inet::Coord getPos() const { return m_pos; }
  inet::Coord getVel() const { return m_vel; }
  cJulian getDate() const { return m_date; }

  virtual void setPos(cVector const &pos) { m_pos = pos; }
  virtual void setVel(cVector const &vel) { m_vel = vel; }
  virtual void setDate(cJulian const &date) { m_date = date; }
  void setUnitsAe() { m_VecUnits = UNITS_AE; }
  void setUnitsKm() { m_VecUnits = UNITS_KM; }
  void setUnitsM() { m_VecUnits = UNITS_M; }
  bool UnitsAreAe() const { return m_VecUnits == UNITS_AE; }
  bool UnitsAreKm() const { return m_VecUnits == UNITS_KM; }
  bool UnitsAreM() const { return m_VecUnits == UNITS_M; }
  void ae2km(); // Convert position, velocity vector units from AE to km

protected:
  void MulPos(double factor) { m_pos = m_pos * factor; }
  void MulVel(double factor) { m_vel = m_vel * factor; }

  enum VecUnits {
    UNITS_NONE, // not initialized
    UNITS_AE,
    UNITS_KM,
    UNITS_M,
  };

  cVector m_pos;
  cVector m_vel;
  cJulian m_date;
  VecUnits m_VecUnits;
};

}  // namespace estnet
