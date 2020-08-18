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

#include "APSKScalarTransmitterWith8PSK.h"

#include <inet/common/Units.h>

#include "PSK8Modulation.h"

namespace estnet {

Define_Module(APSKScalarTransmitterWith8PSK);

void APSKScalarTransmitterWith8PSK::initialize(int stage) {
    // doing the initialization manually, so no exception is thrown
    if (stage == inet::INITSTAGE_LOCAL) {
        if (par("modulation").stdstringValue() != "8PSK") {
            throw omnetpp::cRuntimeError("Only 8PSK modulation is supported");
        }
        modulation = new PSK8Modulation();
        centerFrequency = inet::Hz(
                par("centerFrequency").doubleValueInUnit("Hz"));
        bandwidth = inet::Hz(par("bandwidth").doubleValueInUnit("Hz"));
        preambleDuration = par("preambleDuration");
        headerLength = inet::b(par("headerLength").doubleValueInUnit("b"));
        bitrate = inet::bps(par("bitrate").doubleValueInUnit("bps"));
        power = inet::W(par("power").doubleValueInUnit("W"));
    }
}

}  // namespace estnet
