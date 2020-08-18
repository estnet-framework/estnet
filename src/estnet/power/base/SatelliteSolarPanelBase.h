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

#ifndef __ESTNET_CUBESATPANEL_H_
#define __ESTNET_CUBESATPANEL_H_

#include <omnetpp.h>

#include "estnet/power/contract/ISolarPanel.h"

namespace estnet {

typedef inet::units::value<double, inet::units::units::m2> m2;
typedef inet::units::value<double,
        inet::units::compose<inet::units::units::W, inet::units::pow<m2, -1>>> Wpm2;

/**
 * Describes power production on satellite
 * based on the area of each cell, the sun's intensity,
 * the efficiencies of the system and the power generation
 * of the cells and the number of solar cells on the panel
 *
 * The orientation is modeled by the AntennaMobilty, that
 * models a fixed orientation and position offset to the
 * added to the satellite's * mobility
 */
class ESTNET_API SatelliteSolarPanelBase: public ISolarPanel {
private:
    m2 _cellSize;      // size of one cell
    Wpm2 _sunIntensity; // mean sun intensity
    int _numSolarCells; // amount of solar cells in this panel
    double _refractiveIndex; // the solar panel's surface's refractive index
    W _absoluteSystemLoss; // the absulute system loss of eg diodes on way to battery

    /*
     * Calculates the angle-of-incedence dependency of the solar
     * cell's surface reflectivity according to Frenel's law
     *
     * @param sunAngle: the angle-of-incedence of the sun to the solar
     *                  panel's surface
     * @return double: reflectivity according to Frenel's law
     */
    double calculateReflectivity(rad sunAngle);

protected:
    /** @brief updates the generation and send signal to battery */
    virtual void updatePowerGeneration() override;

    /** Calculate the power generation at the moment.
     *  Therefore, a model that calculates the part of the sun-solarpanel-
     *  vector, that is pointing orthogonal to the panel
     *  Based on this, the power generation is calculated using the sun's
     *  incidence angle a, the efficiencies n, the total area A of the solar cells,
     *  the solar flux B and the reflectivity p:
     *    P = n * B * A * cos(a) * (1 - p)
     *
     *  @return W: current power generation of solar panel in Watt */
    virtual W calculatePowerGeneration() override;

    /** @brief initialization of the module, called by omnet
     *  @param  stage: stage of initialization */
    virtual void initialize(int stage) override;
};

}  // namespace estnet

#endif
