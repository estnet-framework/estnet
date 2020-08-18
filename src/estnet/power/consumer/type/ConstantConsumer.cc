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

#include "ConstantConsumer.h"

using namespace estnet;

Define_Module(ConstantConsumer);

void ConstantConsumer::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_LOCAL) {
        //find battery
        const char *energySourceModule = par("energySourceModule");
        _energySource =
                dynamic_cast<inet::power::IEpEnergySource*>(getModuleByPath(
                        energySourceModule));
        if (!_energySource)
            throw cRuntimeError("Energy source module '%s' not found",
                    energySourceModule);

        //get power consumption from ini file
        this->_consumption = W(this->par("powerConsumption").doubleValue());
        this->_consumption = this->_consumption
                / this->par("efficiency").doubleValue();
        //publish power consumptions once, stays constant whole simulation time
        EV_DEBUG << "Publishing a constant consume of " << _consumption.get()
                        << "from " << par("submoduleName").stringValue()
                        << std::endl;
        emit(inet::power::IEpEnergySource::powerConsumptionChangedSignal,
                _consumption.get());
    } else if (stage == inet::InitStages::INITSTAGE_PHYSICAL_ENVIRONMENT)
        _energySource->addEnergyConsumer(this);
}

int ConstantConsumer::numInitStages() const {
    return inet::InitStages::NUM_INIT_STAGES;
}

inet::power::IEnergySource* ConstantConsumer::getEnergySource() const {
    return _energySource;
}

W ConstantConsumer::getPowerConsumption() const {
    return _consumption;
}

