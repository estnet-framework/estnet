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

package estnet.physicallayer.pathloss;

import inet.physicallayer.pathloss.FreeSpacePathLoss;

import estnet.environment.atmosphere.*;

//
// Path loss, that extends the Free Space Path Loss
// with atmospheric losses
//
module AtmosphericPathLoss extends FreeSpacePathLoss
{
    double fogAttenuation @unit(dB) = default(0.0dB);
    double snowAttenuation @unit(dB) = default(0.0dB);
    @class(AtmosphericPathLoss);
    submodules:
        weatherModel: SimpleWeatherModel{
            
        }
}
