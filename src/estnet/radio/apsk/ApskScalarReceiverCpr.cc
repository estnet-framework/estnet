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

#include "ApskScalarReceiverCpr.h"

namespace estnet {

Define_Module(ApskScalarReceiverCpr);

void ApskScalarReceiverCpr::initialize(int stage) {
    ApskScalarReceiver::initialize(stage);
    // override the snirthreshold when creating the contact plan
    if (stage == inet::INITSTAGE_LOCAL
            && par("useCpCreationParameters").boolValue())
        snirThreshold = inet::math::dB2fraction(par("snirThresholdCpCreation"));
}

} //namespace
