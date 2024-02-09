//
// Copyright (C) 2013 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include "GeographicIsotropicScalarBackgroundNoise.h"

#include <string>

#include "inet/physicallayer/analogmodel/packetlevel/ScalarNoise.h"
#include "inet/physicallayer/backgroundnoise/IsotropicScalarBackgroundNoise.h"
#include "inet/physicallayer/common/packetlevel/BandListening.h"
#include "inet/physicallayer/contract/packetlevel/IAntenna.h"
#include "inet/physicallayer/contract/packetlevel/IRadio.h"

#include "estnet/environment/earthmodel/EarthModelFactory.h"
#include "estnet/mobility/satellite/SatMobility.h"

namespace estnet {

Define_Module(GeographicIsotropicScalarBackgroundNoise);

void GeographicIsotropicScalarBackgroundNoise::initialize(int stage) {
    cModule::initialize(stage);
    if (stage == 0) {
        power = inet::mW(inet::math::dBmW2mW(par("power")));
        interpolation.load(par("noiseMapPath"));
        std::string noiseMapPath = par("noiseMapPath");
        EV_INFO << "using noise model defined in " << noiseMapPath
                       << omnetpp::endl;
        // load Earth model
        earthModel = EarthModelFactory::get(
                EarthModelFactory::EarthModels::WGS84);
    }
}

GeographicIsotropicScalarBackgroundNoise::GeographicIsotropicScalarBackgroundNoise() :
        interpolation() {
}

const inet::physicallayer::INoise*
GeographicIsotropicScalarBackgroundNoise::computeNoise(
        const inet::physicallayer::IListening *listening) const {
    const inet::physicallayer::BandListening *bandListening =
            omnetpp::check_and_cast<const inet::physicallayer::BandListening*>(
                    listening);
    omnetpp::simtime_t startTime = listening->getStartTime();
    omnetpp::simtime_t endTime = listening->getEndTime();

    inet::W localNoisePower;

    // get position
    // inet::Quaternion rxOrientation;
    inet::Coord rxPosition;

    const inet::physicallayer::IAntenna *receiverAntenna =
            listening->getReceiver()->getAntenna();
    if (dynamic_cast<SatMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
    ->getParentModule() // wlan
    ->getParentModule() // networkHost
    ->getSubmodule("mobility")) != nullptr) {
        // Satellite
        SatMobility *RxMobility =
                dynamic_cast<SatMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
                ->getParentModule() // wlan
                ->getParentModule() // networkHost
                ->getSubmodule("mobility"));

        // rxOrientation = RxMobility->getCurrentAngularPosition();
        rxPosition = RxMobility->getCurrentPosition();
        // convert ECI coordinates -> lan, lon, alt
        inet::deg latitude, longitude;
        inet::m altitude;
        earthModel->convertECIToLatLongHeight(
                GlobalJulianDate::getInstance().currentSimTime(), rxPosition,
                latitude, longitude, altitude);

        localNoisePower = inet::W(
                interpolation.get(latitude.get(), longitude.get()));
        EV_TRACE << "SAT_NOISE (" << latitude << ", " << longitude << ", "
                        << altitude << ") = " << localNoisePower
                        << omnetpp::endl;
    } else {
        // orbital noise power distribution applies to satellites only
        // so we just take the constant power parameter value
        // if we are not dealing with SatMobility
        localNoisePower = power;
        EV_TRACE << "GS_NOISE" << omnetpp::endl;
    }

    std::map<omnetpp::simtime_t, inet::W> *powerChanges = new std::map<
            omnetpp::simtime_t, inet::W>();
    powerChanges->insert(
            std::pair<omnetpp::simtime_t, inet::W>(startTime, localNoisePower));
    powerChanges->insert(
            std::pair<omnetpp::simtime_t, inet::W>(endTime, -localNoisePower));
    return new inet::physicallayer::ScalarNoise(startTime, endTime,
            bandListening->getCenterFrequency(), bandListening->getBandwidth(),
            powerChanges);
}

} // namespace estnet
