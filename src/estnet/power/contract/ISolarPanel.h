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

#ifndef ENERGYMODEL_SOLARPANELS_ISOLARPANEL_H_
#define ENERGYMODEL_SOLARPANELS_ISOLARPANEL_H_

#include <omnetpp.h>
#include <inet/common/geometry/common/Coord.h>
#include <inet/common/Units.h>
#include <inet/power/contract/IEpEnergyGenerator.h>
#include <inet/power/contract/IEpEnergySink.h>

#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/antenna/base/AntennaMobility.h"
#include "estnet/power/EnergyModule.h"
#include "estnet/node/satellite/Satellite.h"

namespace estnet {

typedef inet::units::value<double, inet::units::units::rad> rad;
typedef inet::units::value<double, inet::units::units::W> W;
typedef inet::units::value<double, inet::units::units::s> s;
typedef inet::units::value<double,
        inet::units::compose<inet::units::units::W, inet::units::units::s>> Ws;

/*
 * Interface for all different energy producing classes having
 * the behavioral of solar panels providing all methods required,
 * like calculating the sun's incidence angle
 *
 * The orientation is modeled by the AntennaMobilty, that
 * models a fixed orientation and position offset to the
 * added to the satellite's * mobility
 */
class ESTNET_API ISolarPanel: public omnetpp::cSimpleModule,
        public inet::power::IEpEnergyGenerator {
protected:
    inet::power::IEpEnergySink *_energySink = nullptr;  ///< ptr to energy sink
    int _checkIntervall;       ///< recalculation intervall of energy production
    double _efficiency;         ///< solar cell efficiency
    double _systemLosses;       ///< losses of cables, transforming,..
    AntennaMobility *_mobility; ///< z-axis of panel frame is normal vector to panel plane
    EnergyModule *_energyModule; ///< ptr to energy module

    W _currentPower;            ///< power produced at the moment
    omnetpp::cMessage *_checkTimer;

    /** @brief calculates angle between sun vector and solar panel pointing vector
     *  @return rad: sun illumination angle */
    virtual rad getSunAngle();

    /** @brief returns number of initialization stages
     *  @return int: number of initialization stages */
    virtual int numInitStages() const;

    /** @brief initialize function
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) = 0;

    /** @brief scheduled self-messages receiver function
     *  @param  message: message that is received */
    virtual void handleMessage(omnetpp::cMessage *message);

    /** @brief updates the generation and send signal to battery */
    virtual void updatePowerGeneration() = 0;

public:
    /** @brief cancel and delete scheduled messages */
    virtual ~ISolarPanel();

    /** @brief get energy sink that is belonging to the solar panel
     *  @return IEnergySink: battery to which power is transfered */
    virtual inet::power::IEnergySink* getEnergySink() const override;

    /** @brief returns the power produced by the solarpanel
     *  @return W: current power generation of solar panel in Watt */
    virtual W getPowerGeneration() const override;

    /** @brief calculate the power that is produced at the moment */
    virtual W calculatePowerGeneration() = 0;

};

}  // namespace estnet

#endif /* ENERGYMODEL_SOLARPANELS_ISOLARPANEL_H_ */
