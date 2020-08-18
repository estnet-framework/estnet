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

#ifndef UTILS_KDTREEADAPTER_H_
#define UTILS_KDTREEADAPTER_H_

#include <math.h>
#include <map>
#include <algorithm>

#include <inet/common/INETMath.h>
#include <inet/physicallayer/contract/packetlevel/IRadio.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskScalarReceiver.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskScalarTransmitter.h>
#include <inet/physicallayer/apskradio/packetlevel/ApskRadio.h>
#include <inet/common/geometry/common/Coord.h>

#include "nanoflann.hpp"
#include "estnet/node/base/NodeBase.h"
#include "estnet/common/node/NodeRegistry.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/mobility/satellite/propagator/position/PositionPropagatorKepler.h"

using namespace std;
using namespace inet;
using namespace physicallayer;
using namespace units::constants;
using namespace nanoflann;

namespace estnet {

typedef std::vector<const IRadio*> radioList;

// struct representing node in kd-tree
template<typename T, class nodeType>
struct node {
    //storing coordinates of current position
    T x, y, z;
    unsigned int nodeId;
    nodeType radio;
};

//struct needed by nanoflann kd-tree as datastruct
template<typename T, class nodeType>
struct nodePoints {
    std::map<unsigned int, node<T, nodeType>> nodes = std::map<unsigned int,
            node<T, nodeType>>();

    inline T kdtree_get_pt(const size_t idx, int dim) const {
        unsigned int id = static_cast<unsigned int>(idx);
        node<T, nodeType> newNode = nodes.at(id);
        if (dim == 0)
            return newNode.x;
        else if (dim == 1)
            return newNode.y;
        else
            return newNode.z;
    }

    inline size_t kdtree_get_point_count() const {
        return nodes.size();
    }

    template<class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
        return false;
    }
};

/**
 * class for saving radios and nodes in a k-d-Tree
 * using efficient radius search for contact planning
 * based on nanoflann
 */
class ESTNET_API KDTreeAdapter {
    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<long long, nodePoints<long long, const IRadio*> >,
            nodePoints<long long, const IRadio*>, 3 /* dim */, long long> radio_kd_tree_t; ///< KDTreeAdaptor definition for radios

private:
    radio_kd_tree_t *currentTree = nullptr; ///< the radio kdtree is stored in this variable
    nodePoints<long long, const IRadio*> *radioNodePtsPtr = nullptr; ///< list containing all radios with position as node

public:
    ~KDTreeAdapter() {
        delete currentTree;
        delete radioNodePtsPtr;
    }
    /** @brief generate a node storing a radio with current position and node-ID*/
    template<typename T>
    node<T, const IRadio*> generateNode(const IRadio *iRadio) {
        const Radio *radio = check_and_cast<const Radio*>(iRadio);
        auto nodeB = check_and_cast<const NodeBase*>(
                radio->getParentModule()->getParentModule()->getParentModule());
        Coord coordinates = nodeB->getMobility()->getCurrentPosition();
        // the node is fed with the radio's position and node number of the belonging satellite
        node<T, const IRadio*> newNode;
        newNode.x = coordinates.x / 1000;
        newNode.y = coordinates.y / 1000;
        newNode.z = coordinates.z / 1000;
        newNode.radio = radio;
        newNode.nodeId = nodeB->getNodeNo();
        return newNode;
    }

    /** @brief feeds the DatasetAdapter with all radios in nodes*/
    template<typename T>
    void generateCoordinates(nodePoints<T, const IRadio*> *&radioNodePts,
            const radioList radios) {
        for (auto iRadio : radios) {
            //create node with radios position
            node<T, const IRadio*> newNode = generateNode<T>(iRadio);
            radioNodePts->nodes.insert(
                    std::pair<T, node<T, const IRadio*>>(newNode.nodeId - 1,
                            newNode));
        }
    }

    /** @brief generate the kd-Tree with current data of the radios*/
    template<typename num_t>
    void createKDRadioTree(const radioList radios) {
        //free memory and create new empty list
        if (currentTree != nullptr) {
            delete currentTree;
            delete radioNodePtsPtr;
        }
        radioNodePtsPtr = new nodePoints<num_t, const IRadio*>();
        generateCoordinates<num_t>(radioNodePtsPtr, radios);
        currentTree = new radio_kd_tree_t(3 /*dim*/, *radioNodePtsPtr,
                KDTreeSingleIndexAdaptorParams(30 /* max leaf */));
        currentTree->buildIndex();
    }

    /** @brief compute the maximum communication range based on the APSK model or the geometric constraints */
    unsigned int computeCommRadius(const IRadio *iRadio,
            const IRadio *iRadioPartner) {
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
            double N_B = -134; // dBm
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

    /** @brief compute the maximum communication range im kilometer based on the geometric constraints */
    unsigned int calcGeomConstrRadius(const Radio *radio,
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

    /** @brief does an unsorted radius search starting at the given radio and puts result in ret_matches*/
    template<typename num_t>
    void unsortedRadiusSearch(int transmitterId,
            std::vector<std::pair<long long, long long> > &ret_matches,
            long long search_radius) {
        nanoflann::SearchParams params;
        params.sorted = false; //sorted is not required
        int id = transmitterId - 1;
        search_radius += 1 + search_radius * 0.01; //add 1 % on radius for safety (casts from double to int and calculation inaccuracies)
        long long x = currentTree->dataset.kdtree_get_pt(id, 0);
        long long y = currentTree->dataset.kdtree_get_pt(id, 1);
        long long z = currentTree->dataset.kdtree_get_pt(id, 2);
        const long long query_pt[3] = { x, y, z };
        currentTree->radiusSearch(&query_pt[0], search_radius * search_radius,
                ret_matches, params);

    }
};

}  // namespace estnet

#endif /* UTILS_KDTREEADAPTER_H_ */
