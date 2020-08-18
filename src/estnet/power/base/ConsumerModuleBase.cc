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

#include "ConsumerModuleBase.h"

#include "estnet/power/consumer/type/CoupledConsumer.h"

using namespace estnet;

Define_Module(ConsumerModuleBase);

void ConsumerModuleBase::initialize(int stage) {
    if (stage == inet::INITSTAGE_NETWORK_LAYER) {
        //find battery
        const char *energySourceModule = par("energySourceModule");
        _energySource =
                dynamic_cast<inet::power::IEpEnergySource*>(getModuleByPath(
                        energySourceModule));
        if (!_energySource)
            throw cRuntimeError("Energy source module '%s' not found",
                    energySourceModule);

        //intitialize state handler vector
        int num = par("numHandler");
        for (int i = 1; i <= num; i++) {
            char str[14];
            sprintf(str, "stateHandler%d", num);
            this->_stateHandler.push_back(
                    check_and_cast<ConsumerStateHandler*>(
                            this->getSubmodule(str)));
        }
        //get power consumption from state handler
        this->powerConsumptionChanged();
        //publish power consumptions
        EV_DEBUG << "Publishing a start consume of " << _consumption.get()
                        << std::endl;
        emit(inet::power::IEpEnergySource::powerConsumptionChangedSignal,
                _consumption.get());
    } else if (stage == inet::INITSTAGE_TRANSPORT_LAYER) {
        _energySource->addEnergyConsumer(this);
        bool active = false;
        for (auto *handler : this->_stateHandler) {
            if (handler->isActive()) {
                active = true;
            }
        }
        sendStatusToCoupledConsumers(active);
    }
}

int ConsumerModuleBase::numInitStages() const {
    return inet::InitStages::NUM_INIT_STAGES;
}

inet::power::IEnergySource* ConsumerModuleBase::getEnergySource() const {
    return _energySource;
}

W ConsumerModuleBase::getPowerConsumption() const {
    return _consumption;
}

void ConsumerModuleBase::addStateHandler(ConsumerStateHandler *stateHandler) {
    this->_stateHandler.push_back(stateHandler);
}

void ConsumerModuleBase::powerConsumptionChanged() {
    //ask all handlers, what is their proposed consumption and get the maximum
    W maxPower = W(0);
    bool active = false;
    for (auto *handler : this->_stateHandler) {
        if (handler->getProposedPowerConsumption() > maxPower) {
            maxPower = handler->getProposedPowerConsumption();
        }
        if (handler->isActive()) {
            active = true;
        }
    }
    //if consumption of module has changed, emit signal to tell battery that the consumption has changed
    if (maxPower != this->_consumption) {
        this->_consumption = maxPower;
        //publish power consumptions
        EV_DEBUG << "Publishing a consume of " << _consumption.get()
                        << std::endl;
        emit(inet::power::IEpEnergySource::powerConsumptionChangedSignal,
                _consumption.get());
        // update state for any consumers that are coupled on this module
        sendStatusToCoupledConsumers(active);
    }

}

void ConsumerModuleBase::sendStatusToCoupledConsumers(bool on) {
    for (auto consumer : this->_coupledConsumer) {
        CoupledConsumer *cConsumer = check_and_cast<CoupledConsumer*>(consumer);
        cConsumer->updatePowerConsumption(on);
    }
}

void ConsumerModuleBase::addCoupledConsumer(ConsumerStateHandler *consumer) {
    this->_coupledConsumer.push_back(consumer);
}

