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

#include "DirectionalScalarBackgroundNoise.h"

#include <inet/physicallayer/common/packetlevel/BandListening.h>
#include <inet/physicallayer/analogmodel/packetlevel/ScalarNoise.h>
#include <inet/physicallayer/contract/packetlevel/IRadioMedium.h>
#include <inet/physicallayer/antenna/AxiallySymmetricAntenna.h>
#include <inet/physicallayer/antenna/ConstantGainAntenna.h>
#include <inet/physicallayer/antenna/CosineAntenna.h>
#include <inet/physicallayer/antenna/DipoleAntenna.h>
#include <inet/physicallayer/antenna/InterpolatingAntenna.h>
#include <inet/physicallayer/antenna/IsotropicAntenna.h>
#include <inet/physicallayer/antenna/ParabolicAntenna.h>

#include "estnet/node/satellite/Satellite.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/terrestrial/StaticTerrestrialMobility.h"
#include "estnet/global_config.h"

namespace estnet {

Define_Module(DirectionalScalarBackgroundNoise);

DirectionalScalarBackgroundNoise::DirectionalScalarBackgroundNoise() {
}

void DirectionalScalarBackgroundNoise::initialize(int stage) {
    cModule::initialize(stage);
    if (stage == 0) {
        this->receiverNoiseTemp = this->par("receiverNoiseTemp").doubleValue();
        this->t_Earth = this->par("t_Earth").doubleValue();
        this->t_Space = this->par("t_Space").doubleValue();
    }
}

const double DirectionalScalarBackgroundNoise::computeEarthInFOV(
        inet::Coord satPosition, inet::Quaternion satOrientation,
        double beamWidth) const {
    //getting some basic values from the input variables
    inet::Coord point_sat = satOrientation.rotate(inet::Coord::X_AXIS); // satellite pointing vector
    double phi_sat = std::acos(
            (point_sat * (-satPosition))
                    / (point_sat.length() * satPosition.length())); //angle by that point_sat is off from earth pointing mode
    double h_sat = satPosition.length(); // satellite distance to earth center
    double R_E = WGS_84_RADIUS_EQUATOR; // earth radius
    double phi_bw = inet::math::deg2rad(beamWidth / 2); // half the beamwidth in RAD

    if (h_sat < (100000 + R_E)) {
        /*if the satellite is close to earth the curvature doesn't play a significant role and some
         simplifications can be made. We are going to pretend that the satellite is located on the surface
         of a flat earth*/
        double phi_hori = phi_sat - (0.5 * PI); // angle by which the satellite is off the horizon
        //double phi_free = phi_bw + phi_hori; // angle which is not obscured by the horizon
        double phi_obs = phi_bw - phi_hori; // angle  which is obscured by the horizon

        double ratio = phi_obs / (2 * phi_bw);
        return ratio;
    }

    //calculate distance and angle to horizon
    //earth center, the satellite and the horizon form a right triangle
    double d_hori = std::sqrt((h_sat * h_sat) - (R_E * R_E)); // distance to the horizon
    double alpha_hori = std::asin(R_E / h_sat); // angle between the vectors satellite_earthCenter and satellite_horizon
    double phi_hori = phi_sat - alpha_hori; // angle between the vectors point_sat and satellite_horizon

    /*Note: If the satellite points bellow the horizon, phi_hori will be negative. For simplicity's sake the explanation
     below assumes that the satellite is pointing above the horizon, however the end result stays correct for both positive
     and negative phi_hori individually*/

    if (beamWidth >= 360) {
        return alpha_hori / (2.0 * M_PI);
    }
    if (alpha_hori < (phi_sat - phi_bw)) {
        //the satellite is pointing to high to see the horizon
        return 0;
    }
    if (alpha_hori > (phi_sat + phi_bw)) {
        //the satellite is pointing to low to see the horizon
        return 1;
    }

    /*The FOV of the satellite can be imagined as a cone.
     in the next steps we will look at a circle shaped cross section that is orthogonal to point_sat and intersects with
     the horizon.*/
    double d_FOV = d_hori * std::cos(phi_hori); // distance to the relevant cross section
    double r_FOV = d_FOV * std::tan(phi_bw); // radius of the cross section
    /*From the satellite's point of view, the earth looks like a circle, which intersects with the cross section we
     defined in the previous step. We now calculate how much of the radius is obscured by the earth*/

    if (r_FOV >= R_E) {
        // the satellite is far enough away to see the entire earth
        double A_FOV = PI * (r_FOV * r_FOV); // area of the FOV cross section
        double A_E = PI * (R_E * R_E); // area of the earth cross section

        double ratio = A_E / A_FOV;
        return ratio;
    }

    double s_free = d_FOV * std::tan(phi_hori); // free section of the cross section's radius
    double s_obs = r_FOV - s_free; // obscured section of the radius
    double d_E_FOV = R_E + r_FOV - s_obs; // distance between earth_center and the center of the cross section

    /*the center of the earth, the center of the cross section and the two points where the two circles intersect form
     multiple right triangles. Using the law of cosines we can determine the angles between the line connecting the
     center of both circles and the lines going from each center to the intersecting points*/
    double alpha = std::acos(
            ((r_FOV * r_FOV) + (d_E_FOV * d_E_FOV) - (R_E * R_E))
                    / (2 * r_FOV * d_E_FOV)); //angle at fov_center
    double beta = std::acos(
            ((d_E_FOV * d_E_FOV) + (R_E * R_E) - (r_FOV * r_FOV))
                    / (2 * d_E_FOV * R_E)); //angle at earth_center

    double A_kite = d_E_FOV * R_E * std::sin(beta); // area of the kite between the four points
    double A_seg_FOV = alpha * (r_FOV * r_FOV); // area of a segment of the FOV cross section with angle 2alpha
    double A_seg_E = beta * (R_E * R_E); // area of a segment of the earth cross section with angle 2beta
    double A_inter = A_seg_E + A_seg_FOV - A_kite; // area of the intersection
    double A_FOV = PI * (r_FOV * r_FOV); //area of the entire FOV cross section

    double ratio = A_inter / A_FOV;

    return ratio;
}

const double DirectionalScalarBackgroundNoise::computeAntennaNoise(
        const inet::physicallayer::IListening *listening) const {

    inet::Quaternion rxOrientation;
    inet::Coord rxPosition;

    const inet::physicallayer::IAntenna *receiverAntenna =
            listening->getReceiver()->getAntenna();
    if (dynamic_cast<SatMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
    ->getParentModule() // wlan
    ->getParentModule() // networkHost
    ->getSubmodule("mobility")) != nullptr) {
        //Satellite
        SatMobility *RxMobility =
                dynamic_cast<SatMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule()// radio
                ->getParentModule() // wlan
                ->getParentModule() // networkHost
                ->getSubmodule("mobility"));

        rxOrientation = RxMobility->getCurrentAngularPosition();
        rxPosition = RxMobility->getCurrentPosition();

    } else if (dynamic_cast<StaticTerrestrialMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule()// radio
    ->getParentModule() // wlan
    ->getParentModule() // networkHost
    ->getSubmodule("mobility")) != nullptr) {
        //Ground Station
        StaticTerrestrialMobility *RxMobility =
                dynamic_cast<StaticTerrestrialMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule()// radio
                ->getParentModule() // wlan
                ->getParentModule() // networkHost
                ->getSubmodule("mobility"));
        rxOrientation = RxMobility->getCurrentAngularPosition();
        rxPosition = RxMobility->getCurrentPosition();
    }

    //figure out the Antenna type and set the beamwidth accordingly
    double beamWidth = 30.0;
    if (dynamic_cast<const inet::physicallayer::ConstantGainAntenna*>(receiverAntenna)
            != nullptr
            || dynamic_cast<const inet::physicallayer::IsotropicAntenna*>(receiverAntenna)
                    != nullptr) {
        //The Antenna is omnidirectional
        beamWidth = 360;
    } else if (dynamic_cast<const inet::physicallayer::CosineAntenna*>(receiverAntenna)
            != nullptr) {
        const inet::physicallayer::CosineAntenna *receiverAntennaCosine =
                dynamic_cast<const inet::physicallayer::CosineAntenna*>(receiverAntenna);
        beamWidth = double(receiverAntennaCosine->par("beamWidth"));
    } else if (dynamic_cast<const inet::physicallayer::ParabolicAntenna*>(receiverAntenna)
            != nullptr) {
        const inet::physicallayer::ParabolicAntenna *receiverAntennaParabolic =
                dynamic_cast<const inet::physicallayer::ParabolicAntenna*>(receiverAntenna);
        beamWidth = double(receiverAntennaParabolic->par("beamWidth"));
    } else if (dynamic_cast<const inet::physicallayer::DipoleAntenna*>(receiverAntenna)
            != nullptr
            || dynamic_cast<const inet::physicallayer::AxiallySymmetricAntenna*>(receiverAntenna)
                    != nullptr
            || dynamic_cast<const inet::physicallayer::InterpolatingAntenna*>(receiverAntenna)
                    != nullptr) {
        /*These antenna types do not have a beamwidth, we get a rough estimation of the beamwidth
         by searching for where the gain drops by 3dB in increments of 1 deg*/
        inet::Coord probing_vector = inet::Coord::X_AXIS;
        inet::Quaternion probing_quat = inet::Quaternion();
        inet::Quaternion roatationQuat = inet::Quaternion(0, 0.0087265, 0,
                0.9999619); //Quat that rotates a vector around the Y_AXIS by 1 degree
        double maxGain = receiverAntenna->getGain()->getMaxGain();
        double halfGain = maxGain - 3;
        double gain = maxGain;
        double beamWidth_temp = 0;
        while (gain > halfGain && beamWidth_temp < 360) {
            probing_vector = roatationQuat.rotate(probing_vector);
            probing_quat = inet::Quaternion::rotationFromTo(inet::Coord::X_AXIS,
                    probing_vector);
            gain = receiverAntenna->getGain()->computeGain(probing_quat);
            beamWidth_temp++;
        }
        beamWidth = beamWidth_temp;
    } else {
        omnetpp::cRuntimeError("Antenna class is not supported "
                " by directionalScalarBackgroundNoise");
    }

    //calculate antenna noise
    double bandwidth =
            dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
            ->par("bandwidth");

    double EarthInFOV = this->computeEarthInFOV(rxPosition, rxOrientation,
            beamWidth);
    double P_N_earth = t_Earth * K_B * bandwidth;
    double P_N_space = t_Space * K_B * bandwidth;
    double P_N_system = (P_N_earth * EarthInFOV)
            + ((1 - EarthInFOV) * P_N_space);

    P_N_system += receiverNoiseTemp * K_B * bandwidth;

    return P_N_system;
}

const inet::physicallayer::INoise* DirectionalScalarBackgroundNoise::computeNoise(
        const inet::physicallayer::IListening *listening) const {

    inet::units::values::W P_N_antenna = inet::units::values::W(
            this->computeAntennaNoise(listening)); // Antenna noise

    inet::units::values::W powerChange = P_N_antenna;

    //We can not just call IsotropicScalarBackgroundNoise::computeNoise here, because it returns a const INoise
    const inet::physicallayer::BandListening *bandListening =
            omnetpp::check_and_cast<const inet::physicallayer::BandListening*>(
                    listening);
    omnetpp::simtime_t startTime = listening->getStartTime();
    omnetpp::simtime_t endTime = listening->getEndTime();
    std::map<omnetpp::simtime_t, inet::units::values::W> *powerChanges =
            new std::map<omnetpp::simtime_t, inet::units::values::W>();
    powerChanges->insert(
            std::pair<omnetpp::simtime_t, inet::units::values::W>(startTime,
                    powerChange));
    powerChanges->insert(
            std::pair<omnetpp::simtime_t, inet::units::values::W>(endTime,
                    -powerChange));

    return new inet::physicallayer::ScalarNoise(startTime, endTime,
            bandListening->getCenterFrequency(), bandListening->getBandwidth(),
            powerChanges);
}

} //estnet
