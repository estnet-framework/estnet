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

#include "GMSKModulation.h"

namespace estnet {

double GMSKModulation::calculateBER(double snir, inet::Hz bandwidth,
        inet::bps bitrate) const {
    //http://ijarcet.org/wp-content/uploads/IJARCET-VOL-2-ISSUE-4-1389-1392.pdf
    //https://www.unilim.fr/pages_perso/vahid/notes/ber_awgn.pdf
    return (0.5 * erfc(sqrt(0.68 * snir * bandwidth.get() / bitrate.get())));
}

double GMSKModulation::calculateSER(double snir, inet::Hz bandwidth,
        inet::bps bitrate) const {
    return NAN; // not needed for APSKScalarSimulation
}

}  // namespace estnet
