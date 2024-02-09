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
#include "SimpleWeatherModel.h"

#include <fstream>
#include <iostream>
#include <math.h>

#include "estnet/environment/earthmodel/EarthModelFactory.h"

namespace estnet {

Define_Module(SimpleWeatherModel);

void SimpleWeatherModel::initialize() {
    _rainIntensity = par("rainIntensity").doubleValue();
    _rainHeight = inetu::m(par("rainHeight").doubleValueInUnit("m"));
    setUpAandB(par("rainParameterFile").stringValue());
    std::string rainDropDistrStr = par("rainDropDistribution").stdstringValue();
    _rainDropDistribution = getRainDropDistribution(rainDropDistrStr);

    this->_earthModel = EarthModelFactory::get(
            EarthModelFactory::EarthModels::WGS84);
}

double SimpleWeatherModel::calculateRainAttenuation(inetu::Hz f,
        const inet::Coord &positionEci1, const inet::Coord &positionEci2) {

    auto el = _earthModel->calculateElevation(positionEci1, positionEci2);
    inetu::km distance = _rainHeight / cos(inetu::deg(90) - el);


    // buffer last calculation as mostly the same f is requested
    static RainAttenuationAB ab;
    static inetu::Hz lastF = inetu::Hz(0);
    if (lastF != f) {
        for (auto element : _abLookupTable) {
            if (element.first >= f) {
                ab = element.second;
                break;
            }
        }
        lastF = f;
    }
    auto att = ab.a[_rainDropDistribution]
            * std::pow(_rainIntensity, ab.b[_rainDropDistribution]);
    att *= distance.get();
    return inet::math::dB2fraction(att);
}

rainDropDistribution SimpleWeatherModel::getRainDropDistribution(
        std::string name) {
    if (name == "LP_L")
        return LP_L;
    if (name == "LP_H")
        return LP_H;
    if (name == "MP")
        return MP;
    if (name == "J_T")
        return J_T;
    if (name == "J_D")
        return J_D;

    throw omnetpp::cRuntimeError("Invalid Rain Drop Distribution selected!");

}

void SimpleWeatherModel::setUpAandB(const char *filename) {
    std::ifstream fileAB;
    fileAB.open(filename);
    if (!fileAB.is_open()) {
        std::cerr << "Tried to open: " << filename << std::endl;
        throw std::invalid_argument("Could not open rain setting file!");
    }

    std::string wordBuffer;
    char delimiter = ';';
    while (!fileAB.eof()) {
        // a # (0x23) at the start of the line means we'll skip the whole line
        if (fileAB.peek() == 0x23) {
            std::getline(fileAB, wordBuffer);
            continue;
        }
        RainAttenuationAB ab;
        double f;

        // read in frequency value
        std::getline(fileAB, wordBuffer, delimiter);
        f = std::atof(wordBuffer.c_str());

        // read in a values
        for (int i = 0; i < rainDropDistribution::NUM_DISTRIBUTIONS; i++) {
            std::getline(fileAB, wordBuffer, delimiter);
            ab.a[i] = std::atof(wordBuffer.c_str());
        }
        // read in b values
        for (int i = 0; i < rainDropDistribution::NUM_DISTRIBUTIONS; i++) {
            if (i < rainDropDistribution::NUM_DISTRIBUTIONS - 1) {
                std::getline(fileAB, wordBuffer, delimiter);
            } else {
                std::getline(fileAB, wordBuffer, '\n');
            }
            ab.b[i] = std::atof(wordBuffer.c_str());
        }
        // check if we reached eof, otherwise we recycle the last line
        if (fileAB.eof()) {
            break;
        }

        _abLookupTable.insert(std::make_pair(inetu::GHz(f), ab));
    }
}

} //namespace
