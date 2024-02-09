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

#include "ThreeDbBeamwidthAntenna.h"

#include <inet/common/INETMath.h>

#include "estnet/global_config.h"

using namespace inet::math;

namespace estnet {

Define_Module(ThreeDbBeamwidthAntenna);

void ThreeDbBeamwidthAntenna::initialize(int stage) {
    inet::physicallayer::AntennaBase::initialize(stage);
    if (stage == 0) {
        double maxGain = inet::math::dB2fraction(par("maxGain"));
        double minGain = inet::math::dB2fraction(par("minGain"));
        inet::units::values::deg beamWidth = inet::units::values::deg(
                par("beamWidth"));
        gain = inet::makeShared<ThreeDbAntennaGain>(maxGain, minGain,
                beamWidth);
    }
}

std::ostream& ThreeDbBeamwidthAntenna::printToStream(std::ostream &stream,
        int level) const {
    stream << "ThreeDbBeamwidthAntenna";
    stream << ", maxGain = " << gain->getMaxGain();
    stream << ", beamWidth = " << gain->getBeamWidth();
    return inet::physicallayer::AntennaBase::printToStream(stream, level);
}

inet::units::values::deg ThreeDbBeamwidthAntenna::getBeamWidth() {
    return gain->getBeamWidth();
}

ThreeDbBeamwidthAntenna::ThreeDbAntennaGain::ThreeDbAntennaGain(double maxGain,
        double minGain, inet::units::values::deg beamWidth) :
        maxGain(maxGain), minGain(minGain), beamWidth(beamWidth) {
}

double ThreeDbBeamwidthAntenna::ThreeDbAntennaGain::computeGain(
        const inet::Quaternion direction) const {
    // assumes analog model calculated direction in the transmitters
    // coordinate frame

    double angle = acos(
            direction.rotate(inet::Coord::X_AXIS) * inet::Coord::X_AXIS);

    // the gain computation is actually from inet's ParabolAntenna, as the beam
    // width of it is a good enough approximation for our yagi for now
    return maxGain * dB2fraction(-3 * pow(angle / deg2rad(beamWidth.get()), 2));
}

}  // namespace estnet
