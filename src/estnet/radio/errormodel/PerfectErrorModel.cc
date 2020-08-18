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

#include "PerfectErrorModel.h"

namespace estnet {

Define_Module(PerfectErrorModel);

std::ostream& PerfectErrorModel::printToStream(std::ostream &stream,
        int level) const {
    return stream << "PerfectErrorModel";
}

double PerfectErrorModel::computePacketErrorRate(
        const inet::physicallayer::ISnir *snir,
        inet::physicallayer::IRadioSignal::SignalPart part) const {
    return 0.0;
}

double PerfectErrorModel::computeBitErrorRate(
        const inet::physicallayer::ISnir *snir,
        inet::physicallayer::IRadioSignal::SignalPart part) const {
    return 0.0;
}

double PerfectErrorModel::computeSymbolErrorRate(
        const inet::physicallayer::ISnir *snir,
        inet::physicallayer::IRadioSignal::SignalPart part) const {
    return 0.0;
}

}  // namespace estnet
