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

#include "AtmosphericPathLoss.h"

#include "estnet/environment/atmosphere/SimpleWeatherModel.h"

namespace estnet {

Define_Module(AtmosphericPathLoss);

void AtmosphericPathLoss::initialize(int stage) {
    inet::physicallayer::FreeSpacePathLoss::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        _fogAttenuation = inet::math::dB2fraction(par("fogAttenuation"));
        _snowAttenuation = inet::math::dB2fraction(par("snowAttenuation"));
    }
}

double AtmosphericPathLoss::computePathLoss(
        const inetp::ITransmission *transmission,
        const inetp::IArrival *arrival) const {
    auto fspl = PathLossBase::computePathLoss(transmission, arrival);

    // check if the link is a ground-space link
    double rainAttenuation = 1.0;
    double radiusLimit = EARTH_AVG_R + 100000; // 100 km above ground
    if (transmission->getStartPosition().length() < radiusLimit
            || arrival->getEndPosition().length() < radiusLimit) {
        auto narrowbandSignalAnalogModel = omnetpp::check_and_cast<
                const inetp::INarrowbandSignal*>(
                transmission->getAnalogModel());
        inetu::Hz centerFrequency = inetu::Hz(
                narrowbandSignalAnalogModel->getCenterFrequency());

        auto weatherModel = omnetpp::check_and_cast<SimpleWeatherModel*>(
                this->getSubmodule("weatherModel"));
        rainAttenuation = weatherModel->calculateRainAttenuation(
                centerFrequency, transmission->getStartPosition(),
                arrival->getEndPosition());
    }

    return fspl / rainAttenuation / _fogAttenuation / _snowAttenuation;

}

double AtmosphericPathLoss::computePathLoss(inet::mps propagationSpeed,
        inet::Hz frequency, inet::m distance) const {
    auto fspl = FreeSpacePathLoss::computePathLoss(propagationSpeed, frequency,
            distance);

    return fspl;
}

inet::m AtmosphericPathLoss::computeRange(inet::mps propagationSpeed,
        inet::Hz frequency, double loss) const {
    return FreeSpacePathLoss::computeRange(propagationSpeed, frequency, loss);
}

} //namespace
