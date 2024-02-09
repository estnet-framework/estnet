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

#include "APSKScalarReceiverWithGMSK.h"

#include <inet/physicallayer/apskradio/packetlevel/ApskScalarReceiver.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskScalarTransmission.h>
#include <inet/physicallayer/common/packetlevel/BandListening.h>
#include <inet/physicallayer/backgroundnoise/IsotropicScalarBackgroundNoise.h>
#include <inet/physicallayer/analogmodel/packetlevel/ScalarNoise.h>

#include "estnet/physicallayer/noise/DirectionalScalarBackgroundNoise.h"
#include "GMSKModulation.h"


namespace estnet {

Define_Module(APSKScalarReceiverWithGMSK);

omnetpp::simsignal_t APSKScalarReceiverWithGMSK::noisePower = registerSignal(
        "noisePower");

void APSKScalarReceiverWithGMSK::initialize(int stage) {
    // doing the initialization manually, so no exception is thrown
    if (stage == inet::INITSTAGE_LOCAL) {
        if (par("modulation").stdstringValue() != "GMSK") {
            throw omnetpp::cRuntimeError("Only GMSK modulation is supported");
        }
        modulation = new GMSKModulation();
        errorModel =
                dynamic_cast<inet::physicallayer::IErrorModel*>(getSubmodule(
                        "errorModel"));
        energyDetection = inet::mW(
                inet::math::dBmW2mW(
                        par("energyDetection").doubleValueInUnit("dBm")));
        sensitivity = inet::mW(
                inet::math::dBmW2mW(
                        par("sensitivity").doubleValueInUnit("dBm")));
        centerFrequency = inet::Hz(
                par("centerFrequency").doubleValueInUnit("Hz"));
        bandwidth = inet::Hz(par("bandwidth").doubleValueInUnit("Hz"));
        snirThreshold = inet::math::dB2fraction(
                par("snirThreshold").doubleValueInUnit("dB"));
        const char *snirThresholdModeString = par("snirThresholdMode");
        if (!strcmp("min", snirThresholdModeString))
            snirThresholdMode = SnirThresholdMode::STM_MIN;
        else if (!strcmp("mean", snirThresholdModeString))
            snirThresholdMode = SnirThresholdMode::STM_MEAN;
        else
            throw omnetpp::cRuntimeError("Unknown SNIR threshold mode: '%s'",
                    snirThresholdModeString);
    }
}


}  // namespace estnet
