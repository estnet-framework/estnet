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

#include "BackgroundNoiseSelector.h"

namespace estnet {

Define_Module(BackgroundNoiseSelector);

BackgroundNoiseSelector::BackgroundNoiseSelector() {
}
;

void BackgroundNoiseSelector::initialize(int stage) {
    cModule::initialize(stage);
}

const inet::physicallayer::INoise* BackgroundNoiseSelector::computeNoise(
        const inet::physicallayer::IListening *listening) const {
    const inet::physicallayer::IAntenna *receiverAntenna =
            listening->getReceiver()->getAntenna();
    // look up the kind of mobility of this node and redirect the function call to the correct bachgroundNoise class
    if (dynamic_cast<SatMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
    ->getParentModule() // wlan
    ->getParentModule() // networkHost
    ->getSubmodule("mobility")) != nullptr) {
        // Satellite
        return dynamic_cast<inet::physicallayer::IBackgroundNoise*>(this->getSubmodule(
                "satelliteBackgroundNoise"))->computeNoise(listening);

    } else if (dynamic_cast<StaticTerrestrialMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
    ->getParentModule() // wlan
    ->getParentModule() // networkHost
    ->getSubmodule("mobility")) != nullptr) {
        // Ground station
        return dynamic_cast<inet::physicallayer::IBackgroundNoise*>(this->getSubmodule(
                "groundStationBackgroundNoise"))->computeNoise(listening);

    } else {
        throw new cRuntimeError(
                "Mobility in BackgroundNoiseSelector unknown. It is neither SatMobility nor StaticTerrestrialMobility.");
    }
}

}

