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

package estnet.radio.propagation;


import inet.physicallayer.propagation.ConstantSpeedPropagation;


//
// This propagation model computes the propagation time to be proportional to
// the traveled distance, the ratio is determined by the constant propagation
// speed parameter.
//
module ConstantSpeedPropagationWithMovement extends ConstantSpeedPropagation
{
    parameters:
        ignoreMovementDuringTransmission = default(false); // true means that the movement of the transmitter and the receiver during the signal transmission is ignored
        ignoreMovementDuringPropagation = default(false);  // true means that the movement of the transmitter and the receiver during the signal propagation is ignored
        ignoreMovementDuringReception = default(false);    // true means that the movement of the transmitter and the receiver during the signal reception is ignored
        @class(ConstantSpeedPropagationWithMovement);
}


