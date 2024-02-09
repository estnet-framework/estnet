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

#ifndef __ESTNET_POSITIONPROPAGATORKEPLER_H_
#define __ESTNET_POSITIONPROPAGATORKEPLER_H_

#include "estnet/mobility/satellite/contract/IPositionPropagator.h"
#include "estnet/mobility/satellite/common/cEci.h"
#include "state/PropStateECI.h"
#include "state/PropStateKepler.h"

namespace estnet {

/**
 * Implements the SatelliteProp.ned object.
 */
class ESTNET_API PositionPropagatorKepler: public omnetpp::cSimpleModule,
        public PositionPropagator<PropStateECI> {
protected:
    double timeStep;      ///< temporal simulation resolution
private:
    double GetMeanAnomalyFromTrueAnomaly(double v) const; ///< returns the MeanAnomaly in RAD for a given true anomaly
    double GetMeanAnomaly(double time) const; ///< returns the MeanAnomaly in RAD at simulation time t in secondss
    double GetEccentricAnomaly(double time) const; ///< returns the Eccentric Anomaly in RAD for a time
    double GetTrueAnomaly(double time) const; ///< returns the True Anomaly in rad for a given time
    double GetRadiusFromEccentricAnomaly(double EA) const; ///< returns the radius
                                                           ///(km) from earth core
                                                           /// to the satellite at
    /// eccentric anomaly EA
    double GetRadiusFromTrueAnomaly(double TA) const; ///< returns the radius (km)
                                                      /// from earth core to the
    /// satellite at true anomaly
    /// TA
    double GetSpecificAngularmomentum() const;
    double
    eMeSinE(double E) const; ///< hepler method for eccentric anomaly computation
    static double
    NormalizeAngle(double x); ///< x in rad, normalization between -pi/pi
    cEci GetState(double time) const; ///< returns position[m] and velocity [m/s] at given time

    double a;     ///< semi-major axis
    double e;     ///< eccentricity
    double i;     ///< inclination
    double raan;  ///< (big omega) right ascension of the ascending node
    double aop;   ///< (small omega) argument of periapsis
    double v;     ///< initial true anomaly
    double M_ini; ///< initial mean anomaly

protected:
    /**
     * Initializes all the orbit-parameters and the 3D anmimation.
     * Schedules the first initial movement
     * @param stage of the initialization (2 steps needed)
     */
    virtual void initialize() override;

    /**
     * from cModule
     * handler for incomming messages. in this case as timer for the next position
     * update
     * @param msg incomming message
     */
    virtual void handleMessage(omnetpp::cMessage *msg) override;

    /**
     * Propagates the satellite state starting from the current state until the
     * targetTime point in time.
     * @param targetTime point in time for which the state should be propagated
     * @param newState new propagated state
     * Note: Must be implemented by the respective propagators
     */
    virtual void propagateState(cJulian const &targetTime, state_type &newState)
            override;

    /**
     * Returns whether or not an state propagation/update is necessary for the
     * state propagation targetTime, e.g. if the
     * current state is still within accuracy bounds for the propagation target
     * time.
     * @param targetTime target time for which a state update necessity should be
     * checked.
     * @return true, if an state propagation/update is necessary for the
     * targetTime point in time, false otherwise
     * Note: Must be implemented by the respective propagators
     */
    virtual bool needsStateUpdate(cJulian const &targetTime) override;

    /**
     * Returns the orbital period of the current orbit.
     * Implementation for the IPositionPropagator interface
     * @return orbital period in seconds
     */
    virtual double getOrbitalPeriod() const override;

    /**
     * Returns the current distance to the center of earth.
     * @return orbital radius in kilometers
     */
    virtual double getOrbitalRadius(cJulian const &targetTime) const override;
};

}  // namespace estnet

#endif /* __ESTNET_POSITIONPROPAGATORKEPLER_H_ */
