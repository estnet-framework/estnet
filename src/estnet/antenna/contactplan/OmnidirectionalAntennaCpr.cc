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

#include "OmnidirectionalAntennaCpr.h"

#include <inet/common/INETMath.h>

#include "estnet/global_config.h"

using namespace inet::math;

namespace estnet {

Define_Module(OmnidirectionalAntennaCpr);

void OmnidirectionalAntennaCpr::initialize(int stage) {
    AntennaBase::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        // find out which gain parameter name is used
        double maxGain = dB2fraction(par("maxGain").doubleValue());
        double gainpar = std::max(maxGain,
                dB2fraction(par("gain").doubleValue()));
        double baseGain = dB2fraction(par("baseGain").doubleValue());
        gain = inet::makeShared<AntennaGain>(std::max(baseGain, gainpar));
    }
}
}  // namespace estnet
