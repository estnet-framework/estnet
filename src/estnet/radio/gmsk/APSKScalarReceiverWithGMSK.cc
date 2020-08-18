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

#include "estnet/noise/DirectionalScalarBackgroundNoise.h"
#include "GMSKModulation.h"

#include "inet/physicallayer/apskradio/packetlevel/ApskScalarReceiver.h"
#include "inet/physicallayer/apskradio/packetlevel/ApskScalarTransmission.h"
#include "inet/physicallayer/common/packetlevel/BandListening.h"
#include "inet/physicallayer/backgroundnoise/IsotropicScalarBackgroundNoise.h"
#include "inet/physicallayer/analogmodel/packetlevel/ScalarNoise.h"

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

bool APSKScalarReceiverWithGMSK::computeIsReceptionPossible(
        const inet::physicallayer::IListening *listening,
        const inet::physicallayer::ITransmission *transmission) const {
    return inet::physicallayer::ApskScalarReceiver::computeIsReceptionPossible(
            listening, transmission);
}

bool APSKScalarReceiverWithGMSK::computeIsReceptionPossible(
        const inet::physicallayer::IListening *listening,
        const inet::physicallayer::IReception *reception,
        inet::physicallayer::IRadioSignal::SignalPart part) const {
    //collecting data for statistics
    /*const inet::physicallayer::ScalarNoise *noise = dynamic_cast<const inet::physicallayer::ScalarNoise *>(
     dynamic_cast<DirectionalScalarBackgroundNoise* >(
     this->getParentModule() // radio
     ->getParentModule() // wland
     ->getParentModule() // networkHost
     ->getParentModule() // sat
     ->getParentModule() // SpaceTerrestrialNetwork
     ->getSubmodule("radioMedium")
     ->getSubmodule("backgroundNoise"))
     ->computeNoise(listening));

     omnetpp::simtime_t startTime = noise->getStartTime();
     omnetpp::simtime_t endTime = noise->getEndTime();
     inet::units::values::W P_N = noise->computeMaxPower(startTime, endTime);

     const_cast<APSKScalarReceiverWithGMSK *>(this)->emit(noisePower, P_N.get());*/

    return inet::physicallayer::ApskScalarReceiver::computeIsReceptionPossible(
            listening, reception, part);
}

}  // namespace estnet
