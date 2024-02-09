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




#include "KDTreeAdapter.h"


using namespace estnet;


unsigned int KDTreeAdapter::computeCommRadius(const IRadio *iRadio,
        const IRadio *iRadioPartner, RadioMedium* radioMedium) {
    //check if already calculated due to use of IdealRadio
    const Radio *radioCheck = check_and_cast<const physicallayer::Radio*>(
            iRadio);
    const Radio *radioCheckPartner = check_and_cast<
            const physicallayer::Radio*>(iRadioPartner);
    unsigned int search_radius;
    if (strcmp(
            radioCheck->getParentModule()->par("radioType").stringValue(),
            "UnitDiskRadio") == 0) {
        search_radius =
                iRadio->getTransmitter()->getMaxCommunicationRange().get();
    } else {
        //calculate communication radius with physical model
        const FlatReceiverBase *receiverAPSK = check_and_cast<
                const FlatReceiverBase*>(iRadioPartner->getReceiver());
        const FlatTransmitterBase *transmitterAPSK = check_and_cast<
                const FlatTransmitterBase*>(iRadio->getTransmitter());
        const FlatRadioBase *radioAPSK =
                check_and_cast<const FlatRadioBase*>(iRadio);
        const FlatRadioBase *radioAPSKPartner = check_and_cast<
                const FlatRadioBase*>(iRadioPartner);
        // General parameters for UHF
        double SNIR_thr = math::fraction2dB(
                receiverAPSK->getSNIRThreshold()); // dB
        double N_B;
        try{
            N_B = radioMedium->getSubmodule("backgroundNoise")->par("power").doubleValueInUnit("dBm"); // dBm
        } catch(std::exception &e){
            return calcGeomConstrRadius(radioCheck,
                    radioCheckPartner); //km
        }
        double L_o = 0; // dB
        double c = 299792.458; // km/s
        double f = transmitterAPSK->getCenterFrequency().get(); // Hz

        // radio parameters
        Ptr<const IAntennaGain> gain = radioAPSK->getAntenna()->getGain();
        double G_t = math::fraction2dB(gain->getMaxGain()); // dBi
        W P_t_mW = radioAPSK->getTransmitter()->getMaxPower(); // mW
        double P_t = math::fraction2dB(P_t_mW.get()); // dB

        // communication partner parameters
        double sensitivity = 10
                * log10(receiverAPSK->getSensitivity().get()); // dBm
        Ptr<const IAntennaGain> partner_gain =
                radioAPSKPartner->getAntenna()->getGain();
        double G_r = math::fraction2dB(partner_gain->getMaxGain());
        ; // dBi

        double L_p_SNIR = P_t + G_t + G_r + L_o - N_B - SNIR_thr; // dB
        double L_p_SNIR_linear = math::dB2fraction(L_p_SNIR); // linear
        double range_SNIR_DL = (sqrt(L_p_SNIR_linear) * c)
                / (4 * pi.get() * f); // km

        // DL IF range calculation
        double L_p_range_sensitivity_DL = P_t + G_t + G_r + L_o
                - sensitivity; // dB
        double L_p_range_sensitivity_DL_linear = math::dB2fraction(
                L_p_range_sensitivity_DL); // linear
        double range_sensitivity_DL = (sqrt(L_p_range_sensitivity_DL_linear)
                * c) / (4 * pi.get() * f); // km
        //taking the maximum value of sensitivity and SNIR range
        if (range_SNIR_DL < range_sensitivity_DL) {
            search_radius = range_sensitivity_DL;
        } else {
            search_radius = range_SNIR_DL;
        }

    }
    //calculate geometric constrained radius
    unsigned int geomRadius = calcGeomConstrRadius(radioCheck,
            radioCheckPartner); //km
    if (geomRadius < search_radius) {
        search_radius = geomRadius;
    }
    return search_radius; //km
}

unsigned int KDTreeAdapter::calcGeomConstrRadius(const Radio *radio,
        const Radio *radioPartner) {
    //calculate geometric constrained radius
    const NodeBase *nodeBase;
    bool isISL = false;
    if (0
            == strcmp(
                    radio->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                    "estnet::Satellite")) {
        nodeBase =
                check_and_cast<const NodeBase*>(
                        radio->getParentModule()->getParentModule()->getParentModule());
        if (0
                == strcmp(
                        radioPartner->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                        "estnet::Satellite")) {
            isISL = true;
        }
    } else if (0
            == strcmp(
                    radioPartner->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                    "estnet::Satellite")) {
        nodeBase =
                check_and_cast<const NodeBase*>(
                        radioPartner->getParentModule()->getParentModule()->getParentModule());
    }
    //check for linear mobility for testing reasons
    if (nodeBase->getMobility() == nullptr)
        return 1000000000;
    SatMobility *mob = check_and_cast<SatMobility*>(
            nodeBase->getMobility());

    // calculating semi major axis a and eccentricity e
    double a, e;
    if (strcmp(mob->getSubmodule("positionPropagator")->getClassName(),
            "estnet::PositionPropagatorKepler") == 0) {
        const PositionPropagatorKepler *kepler = check_and_cast<
                const PositionPropagatorKepler*>(
                mob->getSubmodule("positionPropagator"));
        e = kepler->par("e");
        a = kepler->par("a");
    } else {
        int T = mob->getOrbitalPeriod();
        Coord r = mob->getCurrentPositionWithoutSignal();
        Coord v = mob->getCurrentVelocity();
        v /= 1000;
        r /= 1000;
        double GM = 398600.4415;
        a = pow((pow((T / (2 * PI)), 2) * GM), (1.0 / 3.0));
        e = ((v % (r % v)) / (GM) - (r / (r.length()))).length();
    }
    unsigned int Re_2 = pow(6371, 2);
    //calculating geometric constraint radius
    unsigned int geomRadius =
            (unsigned int) (sqrt(pow(a + a * e, 2) - Re_2));
    if (isISL)
        geomRadius *= 2;

    return geomRadius; //km
}


