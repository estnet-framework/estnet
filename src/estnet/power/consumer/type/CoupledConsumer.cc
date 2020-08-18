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

#include "CoupledConsumer.h"

using namespace estnet;

Define_Module(CoupledConsumer);

CoupledConsumer::~CoupledConsumer() {

}

void CoupledConsumer::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_PHYSICAL_OBJECT_CACHE) {
        //get parameters from ini file
        this->_consumption = W(this->par("powerConsumption").doubleValue());
        this->_consumption = this->_consumption
                / this->par("efficiency").doubleValue();
        this->_offConsumption = W(
                this->par("offPowerConsumption").doubleValue());
        this->_offConsumption = this->_offConsumption
                / this->par("efficiency").doubleValue();
        const char *consumerModule = par("consumerModule");
        this->_energyConsumer = check_and_cast<ConsumerModuleBase*>(
                getModuleByPath(consumerModule));

        //setting power consumption to off at begin
        this->_powerConsumption = W(0);
    } else if (stage == 6) {
        //add to state handler
        _energyConsumer->addStateHandler(this);
        //add to the coupled consumer
        const char *cConsumerModuleString = par("pathToCoupledConsumer");
        ConsumerModuleBase *cConsumer = check_and_cast<ConsumerModuleBase*>(
                getModuleByPath(cConsumerModuleString));
        cConsumer->addCoupledConsumer(this);
    }
}

void CoupledConsumer::updatePowerConsumption(bool on) {
    //set consumption depending on the current state
    this->_powerConsumption =
            on ? W(this->_consumption) : W(this->_offConsumption);
    EV_DEBUG << "Publishing a coupled consume of " << this->_powerConsumption
                    << std::endl;
    this->_energyConsumer->powerConsumptionChanged();
}
