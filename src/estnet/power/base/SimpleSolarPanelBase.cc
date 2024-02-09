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

#include "SimpleSolarPanelBase.h"

namespace estnet {

Define_Module(SimpleSolarPanelBase);

void SimpleSolarPanelBase::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_LOCAL) {
        // initialize solar panel's parameter
        this->_maxOutputPower = W(this->par("maxOutputPower").doubleValue());
        this->_efficiency = this->par("efficiency").doubleValue();
        this->_checkIntervall = this->par("checkInterval");
        this->_energyModule = omnetpp::check_and_cast<EnergyModule*>(
                this->getParentModule());
        const char *energySinkModule = par("energySinkModule");
        _energySink = dynamic_cast<inet::power::IEpEnergySink*>(getModuleByPath(
                energySinkModule));
        if (!_energySink)
            throw omnetpp::cRuntimeError("Energy sink module '%s' not found",
                    energySinkModule);

        // set panels orientation, using the antenna mobility
//        double roll = this->par("roll").doubleValue() * M_PI / 180;
//        double pitch = this->par("pitch").doubleValue() * M_PI / 180;
//        double yaw = this->par("yaw").doubleValue() * M_PI / 180;
//        auto sat = omnetpp::check_and_cast<Satellite*>(
//                this->getParentModule()->getParentModule());
        this->_mobility = omnetpp::check_and_cast<OffsetMobility*>(getSubmodule("mobility"));

        // start message for power updating
        this->_checkTimer = new omnetpp::cMessage("checkPowerCreationTimer");
        this->scheduleAt(this->_checkIntervall, this->_checkTimer);
    } else if (stage == inet::InitStages::INITSTAGE_LAST) {
        _energySink->addEnergyGenerator(this);
    }
}

W SimpleSolarPanelBase::calculatePowerGeneration() {
    // check whether satellite is in Eclipse
    if (_energyModule->isInEclipse())
        return W(0);

    // otherwise calculate the angle between sun and solar panel
    rad sunAngle = this->getSunAngle();
    EV_DEBUG << "Sun Angle:" << sunAngle.get() * 180 / M_PI << std::endl;

    // if sun is shining at the upper side of the panel, calculate the power production
    if ((sunAngle.get() > (-M_PI / 2)) && (sunAngle.get() < (M_PI / 2))) {
        // cosine model depending on how close the sun angle is to 90 deg
        double sunEfficiency = cos(sunAngle.get());
        // calculate power produced
        W power = this->_maxOutputPower;
        power *= sunEfficiency;
        power *= this->_efficiency;
        return power;
    } else
        return W(0);
}

void SimpleSolarPanelBase::updatePowerGeneration() {
    // calculate the power that is generated at the moment and publish it to the battery
    this->_currentPower = this->calculatePowerGeneration();
    EV_DEBUG << "Power Generation: " << _currentPower << std::endl;
    emit(inet::power::IEpEnergySink::powerGenerationChangedSignal,
            _currentPower.get());
}

}  // namespace estnet
