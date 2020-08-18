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

#include "AxiallySymmetricAntennaExtraStatistics.h"

#include <inet/common/Units.h>

#include "estnet/global_config.h"

namespace estnet {

Define_Module(AxiallySymmetricAntennaExtraStatistics);

omnetpp::simsignal_t AxiallySymmetricAntennaExtraStatistics::gainSiganl =
        registerSignal("gain");

void AxiallySymmetricAntennaExtraStatistics::initialize(int stage) {
    inet::physicallayer::AxiallySymmetricAntenna::initialize(stage);
    if (stage == inet::INITSTAGE_LOCAL) {
        gain = inet::makeShared<AntennaGainExtraStatistics>(this,
                par("axisOfSymmetry"), inet::math::dB2fraction(par("baseGain")),
                par("gains"));
    }
}

AxiallySymmetricAntennaExtraStatistics::AntennaGainExtraStatistics::AntennaGainExtraStatistics(
        AxiallySymmetricAntennaExtraStatistics *parent, const char *axis,
        double baseGain, const char *gains) :
        minGain(NaN), maxGain(NaN) {
    this->parent = parent;
    axisOfSymmetryDirection = inet::Coord::parse(axis);
    inet::cStringTokenizer tokenizer(gains);
    while (tokenizer.hasMoreTokens()) {
        const char *angleString = tokenizer.nextToken();
        const char *gainString = tokenizer.nextToken();
        if (!angleString || !gainString)
            throw omnetpp::cRuntimeError("Insufficient number of values");
        auto angle = inet::deg(atof(angleString));
        double gain = baseGain * inet::math::dB2fraction(atof(gainString));
        if (std::isnan(minGain) || gain < minGain)
            minGain = gain;
        if (std::isnan(maxGain) || gain > maxGain)
            maxGain = gain;
        gainMap.insert(std::pair<inet::rad, double>(angle, gain));
    }
    if (gainMap.find(inet::deg(0)) == gainMap.end())
        throw omnetpp::cRuntimeError("The first angle must be 0");
    if (gainMap.find(inet::deg(180)) == gainMap.end())
        throw omnetpp::cRuntimeError("The last angle must be 180");
}

double AxiallySymmetricAntennaExtraStatistics::AntennaGainExtraStatistics::computeGain(
        const inet::Quaternion direction) const {
    double normedScalarProduct = direction.rotate(inet::Coord::X_AXIS)
            * inet::Coord::X_AXIS;
    bool checkForInvalidData = normedScalarProduct > 1.0
            || normedScalarProduct < -1.0;
    inet::rad angle;
    if (checkForInvalidData) {
        // catch small calculation errors
        angle = inet::rad((normedScalarProduct > 0 ? acos(1) : acos(-1)));
    } else {
        angle = inet::rad(acos(normedScalarProduct));
    }

    // NOTE: 0 and M_PI are always in the map
    std::map<inet::rad, double>::const_iterator lowerBound =
            gainMap.lower_bound(angle);
    std::map<inet::rad, double>::const_iterator upperBound =
            gainMap.upper_bound(angle);
    if (lowerBound->first != angle)
        lowerBound--;
    if (upperBound == gainMap.end())
        upperBound--;
    if (upperBound == lowerBound) {
        parent->emit(parent->gainSiganl, lowerBound->second);
        return lowerBound->second;
    } else {
        auto lowerAngle = lowerBound->first;
        auto upperAngle = upperBound->first;
        double lowerGain = lowerBound->second;
        double upperGain = upperBound->second;
        double alpha = inet::unit(
                (angle - lowerAngle) / (upperAngle - lowerAngle)).get();
        double gain = (1 - alpha) * lowerGain + alpha * upperGain;
        parent->emit(parent->gainSiganl, gain);
        return gain;
    }
}

}
