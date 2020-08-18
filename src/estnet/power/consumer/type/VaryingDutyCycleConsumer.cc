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

#include "VaryingDutyCycleConsumer.h"

using namespace estnet;

Define_Module(VaryingDutyCycleConsumer);

VaryingDutyCycleConsumer::~VaryingDutyCycleConsumer() {
    cancelAndDelete(this->_changeStatus);

}

void VaryingDutyCycleConsumer::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_PHYSICAL_OBJECT_CACHE) {
        //get parameters from ini file
        this->_consumption = W(this->par("powerConsumption").doubleValue());
        this->_consumption = this->_consumption
                / this->par("efficiency").doubleValue();
        this->_offConsumption = W(
                this->par("offPowerConsumption").doubleValue());
        this->_offConsumption = this->_offConsumption
                / this->par("efficiency").doubleValue();
        this->_onTime = s(this->par("MTA").doubleValue());
        this->_offTime = s(this->par("MTR").doubleValue());
        this->_std = this->par("std").doubleValue();
        _energyConsumer = check_and_cast<ConsumerModuleBase*>(
                getParentModule());

        //publish power consumptions once, starting in state on
        this->_on = false;
        updatePowerConsumption();

        //starting timer for turning off
        this->_changeStatus = new cMessage("switchStatus");
        scheduleAt(this->_onTime.get(), this->_changeStatus);
    } else if (stage == 6)
        _energyConsumer->addStateHandler(this);
}

void VaryingDutyCycleConsumer::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        //switch state and plan when to switch the next time using a normal distribution
        if (this->_on) {
            double nextSchedule = truncnormal(this->_offTime.get(), this->_std);
            scheduleAt(simTime() + SimTime(nextSchedule, SIMTIME_S), msg);
            this->updatePowerConsumption();
        } else {
            double nextSchedule = truncnormal(this->_onTime.get(), this->_std);
            scheduleAt(simTime() + SimTime(nextSchedule, SIMTIME_S), msg);
            this->updatePowerConsumption();
        }
    }
}

void VaryingDutyCycleConsumer::updatePowerConsumption() {
    //toggle state and update the power consumption
    this->_on = !this->_on;
    this->_powerConsumption =
            this->_on ? W(this->_consumption.get()) : W(this->_offConsumption);
    EV_DEBUG << "Publishing a varying dc consume of " << this->_powerConsumption
                    << std::endl;
    this->_energyConsumer->powerConsumptionChanged();
}

