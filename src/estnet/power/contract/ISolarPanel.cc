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

#include "ISolarPanel.h"

using namespace estnet;

ISolarPanel::~ISolarPanel() {
    cancelAndDelete(this->_checkTimer);

}

void ISolarPanel::handleMessage(omnetpp::cMessage *message) {
    if (message == this->_checkTimer) {
        //publish energy produced
        updatePowerGeneration();
        this->scheduleAt(omnetpp::simTime() + this->_checkIntervall,
                this->_checkTimer);
    }
}

int ISolarPanel::numInitStages() const {
    return inet::InitStages::NUM_INIT_STAGES;
}

inet::power::IEnergySink* ISolarPanel::getEnergySink() const {
    return this->_energySink;
}

W ISolarPanel::getPowerGeneration() const {
    return _currentPower;
}

rad ISolarPanel::getSunAngle() {
    auto energyModule = omnetpp::check_and_cast<EnergyModule*>(
            this->getParentModule());

    // calculate sun vector
    inet::Coord vecSatSun = energyModule->getSunPosition()
            - this->_mobility->getCurrentPosition();
    vecSatSun = vecSatSun / 100000000.0; //scaling to prevent overflow

    // calculate angle between solarPanel and sun vector
    inet::Coord normalVecPanelInWorldFrame; //= inet::Coord();
    inet::Quaternion panelOrientation =
            this->_mobility->getCurrentAngularPosition();
    inet::Quaternion panelOrientationConjugated =
            this->_mobility->getCurrentAngularPosition();
    panelOrientationConjugated.conjugate();
    inet::Quaternion zAxis = inet::Quaternion(0, 0, 0, 1);
    normalVecPanelInWorldFrame = (panelOrientation * zAxis
            * panelOrientationConjugated).getV();
    EV_DEBUG << "Normal vector: \nx: " << normalVecPanelInWorldFrame.x << " y: "
                    << normalVecPanelInWorldFrame.y << " z: "
                    << normalVecPanelInWorldFrame.z << " Lenght: "
                    << normalVecPanelInWorldFrame.length() << std::endl;

    //double angle = normalVecPanelInWorldFrame.angle(vecSatSun);
    double normedScalarProduct = vecSatSun * normalVecPanelInWorldFrame
            / normalVecPanelInWorldFrame.length() / vecSatSun.length();
    bool checkForInvalidData = normedScalarProduct > 1.0
            || normedScalarProduct < -1.0;
    double angle;
    if (checkForInvalidData) {
        // catch small calculation errors
        angle = (normedScalarProduct > 0 ? acos(1) : acos(-1));
    } else {
        angle = acos(normedScalarProduct);
    }

    EV_DEBUG << "Angle:" << angle << omnetpp::endl;
    return rad(angle);
}
