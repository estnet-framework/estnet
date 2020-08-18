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

#include "ConstantDutyCycleConsumer.h"

using namespace estnet;

Define_Module(ConstantDutyCycleConsumer);

ConstantDutyCycleConsumer::~ConstantDutyCycleConsumer() {
    cancelAndDelete(this->_changeStatus);

}

void ConstantDutyCycleConsumer::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_PHYSICAL_OBJECT_CACHE) {
        //get parameters from ini file
        this->_consumption = W(this->par("powerConsumption").doubleValue());
        this->_consumption = this->_consumption
                / this->par("efficiency").doubleValue();
        this->_offConsumption = W(
                this->par("offPowerConsumption").doubleValue());
        this->_offConsumption = this->_offConsumption
                / this->par("efficiency").doubleValue();
        this->_onTime = s(this->par("onTime").doubleValue());
        this->_offTime = s(this->par("offTime").doubleValue());
        const char *consumerModule = par("consumerModule");
        this->_energyConsumer = check_and_cast<ConsumerModuleBase*>(
                getModuleByPath(consumerModule));

        //publish power consumptions once, starting in state on
        this->_on = false;
        updatePowerConsumption();

        //starting timer for turning off
        this->_changeStatus = new cMessage("switchStatus");
        scheduleAt(this->_onTime.get(), this->_changeStatus);
    } else if (stage == 5)
        _energyConsumer->addStateHandler(this);
}

void ConstantDutyCycleConsumer::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        //schedule next message and switch state
        if (this->_on) {
            scheduleAt(simTime() + SimTime(this->_offTime.get(), SIMTIME_S),
                    msg);
            this->updatePowerConsumption();
        } else {
            scheduleAt(simTime() + SimTime(this->_onTime.get(), SIMTIME_S),
                    msg);
            this->updatePowerConsumption();
        }
    }
}

void ConstantDutyCycleConsumer::updatePowerConsumption() {
    //toggle state and update the power consumption
    this->_on = !this->_on;
    this->_powerConsumption =
            this->_on ? W(this->_consumption) : W(this->_offConsumption);
    EV_DEBUG << "Publishing a constant dc consume of "
                    << this->_powerConsumption << std::endl;
    this->_energyConsumer->powerConsumptionChanged();
}
