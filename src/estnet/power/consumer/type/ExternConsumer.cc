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

#include "ExternConsumer.h"

using namespace estnet;

Define_Module(ExternConsumer);

void ExternConsumer::initialize(int stage) {
    throw cRuntimeError(
            "The consumer type Extern Consumer is not implemented yet!");
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
        subscribeTopic("/omnet/power/consumption");
    } else if (stage == inet::InitStages::INITSTAGE_PHYSICAL_ENVIRONMENT) {
    }
    _energySource->addEnergyConsumer(this);
}

void ExternConsumer::receivedPubSubMessage(PubSubMsg *pubSubMsg) {
    double dPower = atof((pubSubMsg->value).c_str());
    this->_consumption = W(dPower);
    //publish power consumption
    EV << "Publishing a extern consume of " << _consumption.get() << std::endl;
    emit(inet::power::IEpEnergySource::powerConsumptionChangedSignal,
            _consumption.get());
}

int ExternConsumer::numInitStages() const {
    return inet::InitStages::NUM_INIT_STAGES;
}

inet::power::IEnergySource* ExternConsumer::getEnergySource() const {
    return _energySource;
}

W ExternConsumer::getPowerConsumption() const {
    return _consumption;
}
