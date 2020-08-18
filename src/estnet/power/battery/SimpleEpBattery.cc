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

#include "SimpleEpBattery.h"

using namespace estnet;

Define_Module(SimpleEpBattery);

using namespace inet;
using namespace power;

void SimpleEpBattery::removeEnergyPortion(J energy) {
    EV_DEBUG << "Removed Energy Portion of: " << energy.get() << "J."
                    << std::endl;
    this->setResidualCapacity(this->residualCapacity - energy);
}

void SimpleEpBattery::scheduleTimer() {
    inet::units::values::W totalPower = totalPowerGeneration
            - totalPowerConsumption;
    targetCapacity = residualCapacity;
    if (totalPower > inet::units::values::W(0)) {
        targetCapacity =
                std::isnan(printCapacityStep.get()) ?
                        nominalCapacity :
                        ceil(unit(residualCapacity / printCapacityStep).get())
                                * printCapacityStep;
        // NOTE: make sure capacity will change over time despite double arithmetic
        simtime_t remainingTime;
        if (unit(
                (targetCapacity - residualCapacity) / totalPower
                        / inet::units::values::s(1)).get() < 10e5) {
            remainingTime = unit(
                    (targetCapacity - residualCapacity) / totalPower
                            / inet::units::values::s(1)).get();
        } else {
            remainingTime = 10e5;
        }
        if (remainingTime == 0)
            targetCapacity += printCapacityStep;
    } else if (totalPower < inet::units::values::W(0)) {
        targetCapacity =
                std::isnan(printCapacityStep.get()) ?
                        J(0) :
                        floor(unit(residualCapacity / printCapacityStep).get())
                                * printCapacityStep;
        // make sure capacity will change over time despite double arithmetic
        simtime_t remainingTime;
        if (unit(
                (targetCapacity - residualCapacity) / totalPower
                        / inet::units::values::s(1)).get() < 10e5) {
            remainingTime = unit(
                    (targetCapacity - residualCapacity) / totalPower
                            / inet::units::values::s(1)).get();
        } else {
            remainingTime = 10e5;
        }
        if (remainingTime == 0)
            targetCapacity -= printCapacityStep;
    }
    // enforce target capacity to be in range
    if (targetCapacity < J(0))
        targetCapacity = J(0);
    else if (targetCapacity > nominalCapacity)
        targetCapacity = nominalCapacity;
    simtime_t remainingTime;
    if (unit(
            (targetCapacity - residualCapacity) / totalPower
                    / inet::units::values::s(1)).get() < 10e5) {
        remainingTime = unit(
                (targetCapacity - residualCapacity) / totalPower
                        / inet::units::values::s(1)).get();
    } else {
        remainingTime = 10e5;
    }
    if (timer->isScheduled())
        cancelEvent(timer);
    // don't schedule if there's no progress
    if (remainingTime > 0)
        scheduleAt(simTime() + remainingTime, timer);
}

