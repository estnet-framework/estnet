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

#include "JammingStation.h"

namespace estnet {

Define_Module(JammingStation);

void JammingStation::initialize(int stage) {
    GroundLabel::initialize(stage);
    if (stage == 1) {
        this->earthModel = EarthModelFactory::get(
                EarthModelFactory::EarthModels::WGS84);
        this->longitude = inetu::deg(
                this->par("longitude").doubleValueInUnit("deg"));
        this->latitude = inetu::deg(
                this->par("latitude").doubleValueInUnit("deg"));
        this->minElevation = inetu::deg(
                this->par("minElevation").doubleValueInUnit("deg"));
        this->probability = this->par("jammingProbability");
        this->centerFrequency = inetu::Hz(
                this->par("centerFrequency").doubleValueInUnit("Hz"));
        this->bandwidth = inetu::Hz(
                this->par("bandwidth").doubleValueInUnit("Hz"));
    }
}

double JammingStation::getProbability() {
    return this->probability;
}

inetu::Hz JammingStation::getBandwidth() {
    return this->bandwidth;
}

inetu::Hz JammingStation::getCenterFrequency() {
    return this->centerFrequency;
}

bool JammingStation::isInJammingRange(inet::Coord &coordinates) {
    inetu::deg elevation = this->earthModel->calculateElevation(coordinates,
            latitude, longitude, inetu::m(0));

    return (elevation > minElevation);
}

}  // namespace estnet
