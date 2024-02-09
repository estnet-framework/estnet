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

#include <inet/common/packet/chunk/BitCountChunk.h>
#include <inet/common/ProtocolTag_m.h>
#include <inet/physicallayer/common/packetlevel/Interference.h>
#include <inet/physicallayer/base/packetlevel/PropagationBase.h>

#include "../node/groundstation/GroundStation.h"
#include "ContactPlanCreatingRadioMedium.h"
#include "writer/ContactPlanWriter.h"
#include "iterator/ContactPlanIterator.h"
#include "writer/InterferencePlanWriter.h"
#include "estnet/common/node/NodeRegistry.h"
#include "estnet/common/StlUtils.h"
#include "estnet/common/OmnetUtils.h"

namespace estnet {

static const int64_t LOOPBACK_BITRATE = 100 * 1024 * 1024; // 100 mbit/s
static const int64_t CONNECTED_GROUND_STATION_BITRATE = 100 * 1024 * 1024; // 100 mbit/s
static const int64_t MIN_RANGE = 1;

Define_Module(ContactPlanCreatingRadioMedium);

void ContactPlanCreatingRadioMedium::initialize(int stage) {
    inetp::RadioMedium::initialize(stage);
    if (stage == 0) {
        this->_kdTreeAdapter = new KDTreeAdapter();
    } else if (stage == 1) {
        this->_checkInterval = par("checkInterval");
        this->_assumeBidirectionalSatContacts = par(
                "assumeBidirectionalSatContacts");
        this->_considerAddedInterferences = par("considerAddedInterferences");
        this->_buildInterferencePlan = par("buildInterferencePlan");
        this->_checkISL = par("checkISL");
        this->_checkDownlinks = par("checkDownlinks");
        this->_checkUplinks = par("checkUplinks");

        if (this->_assumeBidirectionalSatContacts) {
            std::cout << "Assuming bidirectional satellite contacts"
                    << std::endl;
        }
        radioModeFilter = par("radioModeFilter");
        listeningFilter = par("listeningFilter");
        macAddressFilter = par("macAddressFilter");
        removeNonInterferingTransmissionsTimer = new omnetpp::cMessage(
                "removeNonInterferingTransmissions");
        this->_checkTimer = new omnetpp::cMessage("contactCheckTimer");
        this->scheduleAt(0, this->_checkTimer);
    }
    if (stage == this->numInitStages() - 1) {
        this->_gsRadiosRadius = new radius_map();
        this->_satRadiosRadius = new radius_map();
        this->_radiosInterferenceRadius = new radius_map();
        const IRadio *satRadioForCalc = 0x00;
        const inetp::Radio *satRadioForInt;
        // searching for one radio of any satellite to be able to calculate gs->sat uplink radius
        for (const IRadio *iRadio : this->radios) {
            const inetp::Radio *radio = check_and_cast<const inetp::Radio*>(
                    iRadio);
            if (strcmp(
                    radio->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                    "estnet::Satellite") == 0) {
                satRadioForCalc = iRadio;
                satRadioForInt = radio;
                break;
            }
        }

        //calculate and save the communication range for the radius search
        for (const IRadio *iRadio : this->radios) {
            const inetp::Radio *radio = check_and_cast<const inetp::Radio*>(
                    iRadio);
            if (strcmp(
                    radio->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                    "estnet::Satellite") == 0) {
                const unsigned int islRadius =
                        this->_kdTreeAdapter->computeCommRadius(iRadio, iRadio, this);
                this->_satRadiosRadius->insert(
                        std::pair<const inetp::IRadio*, unsigned int>(iRadio,
                                islRadius));
            } else {
                unsigned int gsSatRadius =
                        this->_kdTreeAdapter->computeCommRadius(iRadio,
                                satRadioForCalc, this);
                this->_gsRadiosRadius->insert(
                        std::make_pair(iRadio, gsSatRadius));
            }
        }
        //calculate and save the interference ranges, due to orbit constraints
        for (const IRadio *iRadio : this->radios) {
            const inetp::Radio *radio = check_and_cast<const inetp::Radio*>(
                    iRadio);
            const unsigned int radius =
                    this->_kdTreeAdapter->calcGeomConstrRadius(radio,
                            satRadioForInt);
            this->_radiosInterferenceRadius->insert(
                    std::pair<const inetp::IRadio*, unsigned int>(iRadio,
                            radius));
        }
    }
}

template<typename T>
std::vector<T*> ContactPlanCreatingRadioMedium::getVectorOfContacts(
        const contact_map<T> &contacts) const {
    std::vector<T*> rtn;
    for (const contact_map_entry<T> &sourceMapEntry : contacts) {
        for (const contact_map_entry_entry<T> &sinkMapEntry : sourceMapEntry.second) {
            for (T *contact : sinkMapEntry.second) {
                if (contact->enabled
                        && contact->startTime != contact->endTime) {
                    rtn.push_back(contact);
                }
            }
        }
    }
    return rtn;
}

template<typename T>
std::vector<T*> ContactPlanCreatingRadioMedium::getVectorOfInterferingContacts(
        const interference_map_inner<T> &contacts) const {
    std::vector<T*> rtn;
    for (const interference_map_entry_entry<T> &sourceMapEntry : contacts) {
        for (const interference_map_entry_entry_entry<T> &sinkMapEntry : sourceMapEntry.second) {
            for (T *contact : sinkMapEntry.second) {
                //if (contact->enabled && contact->startTime != contact->endTime) {
                rtn.push_back(contact);
                //}
            }
        }
    }
    return rtn;
}

void ContactPlanCreatingRadioMedium::addLoopbackContacts(
        std::vector<contact_plan_entry_t*> &contacts, int64_t totalEndTime) {
    const NodeRegistry *nodeRegistry = NodeRegistry::getInstance();
    std::vector<NodeBase*> nodes = nodeRegistry->getNodes();
    for (const NodeBase *node : nodes) {
        unsigned int nodeNo = node->getNodeNo();
        contact_plan_entry_t *newContactPlanEntry = new contact_plan_entry_t;
        memset(newContactPlanEntry, 0, sizeof(contact_plan_entry_t));
        newContactPlanEntry->sourceNodeId = nodeNo;
        newContactPlanEntry->sinkNodeId = nodeNo;
        newContactPlanEntry->sourceIsGroundStation =
                nodeRegistry->isGroundStation(nodeNo);
        newContactPlanEntry->sinkIsGroundStation =
                nodeRegistry->isGroundStation(nodeNo);
        newContactPlanEntry->startTime = 0;
        newContactPlanEntry->endTime = totalEndTime;
        newContactPlanEntry->bitrate = LOOPBACK_BITRATE;
        newContactPlanEntry->range = MIN_RANGE; // 1 is the minimum
        newContactPlanEntry->minDistance = 0;
        newContactPlanEntry->maxDistance = 0;
        newContactPlanEntry->enabled = true;
        contacts.push_back(newContactPlanEntry);
    }
}

void ContactPlanCreatingRadioMedium::addInternetGroundStationContacts(
        std::vector<contact_plan_entry_t*> &contacts, int64_t totalEndTime) {
    std::vector<GroundStation*> groundStations =
            NodeRegistry::getInstance()->getGroundStations();
    for (const GroundStation *groundStationA : groundStations) {
        for (const GroundStation *groundStationB : groundStations) {
            if (groundStationA != groundStationB
                    && groundStationA->canCommunicateWithoutRadioWith(
                            groundStationB->getNodeNo())) {
                contact_plan_entry_t *newContactPlanEntry =
                        new contact_plan_entry_t;
                memset(newContactPlanEntry, 0, sizeof(contact_plan_entry_t));
                newContactPlanEntry->sourceNodeId = groundStationA->getNodeNo();
                newContactPlanEntry->sinkNodeId = groundStationB->getNodeNo();
                newContactPlanEntry->sourceIsGroundStation = true;
                newContactPlanEntry->sinkIsGroundStation = true;
                newContactPlanEntry->startTime = 0;
                newContactPlanEntry->endTime = totalEndTime;
                newContactPlanEntry->bitrate = CONNECTED_GROUND_STATION_BITRATE;
                newContactPlanEntry->range = 1; // 1 is the minimum
                newContactPlanEntry->minDistance = 0;
                newContactPlanEntry->maxDistance = 0;
                newContactPlanEntry->enabled = true;
                contacts.push_back(newContactPlanEntry);
            }
        }
    }
}

interference_map_inner_inner<interference_plan_entry_t> ContactPlanCreatingRadioMedium::invertInterferenceMap(
        interference_map<interference_plan_entry_t> &interferences) const {
    // we've saved that contact W is interfered by contacts X, Y Z
    // we want to have contact X interfers contact W, ...
    interference_map_inner_inner<interference_plan_entry_t> invertedInterferenceMap;
    for (interference_map_entry<interference_plan_entry_t> &interferencesEntry : interferences) {
        for (interference_map_entry_entry<interference_plan_entry_t> &interferencesEntryEntry : interferencesEntry.second) {
            for (interference_map_entry_entry_entry<interference_plan_entry_t> &interferencesEntryEntryEntry : interferencesEntryEntry.second) {
                //unsigned int sourceNodeId = interferencesEntry.first;
                //unsigned int sinkNodeId = interferencesEntryEntry.first;
                std::vector<unsigned int> interferingNodeId =
                        interferencesEntryEntryEntry.first;
                for (interference_plan_entry_t *interferenceEntry : interferencesEntryEntryEntry.second) {
                    if (invertedInterferenceMap.find(interferingNodeId)
                            == invertedInterferenceMap.end()) {
                        invertedInterferenceMap.emplace(interferingNodeId,
                                InterferencePlan());
                    }
                    invertedInterferenceMap[interferingNodeId].push_back(
                            interferenceEntry);
                }
            }
        }
    }
    return invertedInterferenceMap;
}

void ContactPlanCreatingRadioMedium::finish() {
    // will be freed by realRadioMedium
    inetp::RadioMedium::finish();

    // locations for contact & interference plans
    std::string fullContactPlan(
            omnetpp::getEnvir()->getConfig()->substituteVariables(
                    "${resultdir}/contact_plan-${configname}-${runnumber}.txt"));
    std::string fullInterferencePlan(
            omnetpp::getEnvir()->getConfig()->substituteVariables(
                    "${resultdir}/interference_plan-${configname}-${runnumber}.txt"));

    // write out full contact plan
    std::vector<contact_plan_entry_t*> contacts = this->getVectorOfContacts(
            this->_contacts);

    inetu::s simtime_limit = parseSimTimeLimit(
            cSimulation::getActiveEnvir()->getConfig()->getConfigValue(
                    "sim-time-limit"));

    this->addLoopbackContacts(contacts, simtime_limit.get());
    this->addInternetGroundStationContacts(contacts, simtime_limit.get());

    ContactPlanWriter().write(contacts, fullContactPlan,
            (int64_t) simtime_limit.get());

    std::cout << "Max sim-time: " << (int64_t) simtime_limit.get() << std::endl;
    std::cout << std::endl;
    std::cout << "Full contact plan written to " << fullContactPlan
            << std::endl;

    // write out full interference plan
    if(_buildInterferencePlan){
        InterferencePlan interferences;
        for (interference_map_entry<interference_plan_entry_t> &interferencesEntry : this->_interferences) {
            InterferencePlan interferencesTmp =
                    this->getVectorOfInterferingContacts(interferencesEntry.second);
            interferences.insert(interferences.end(), interferencesTmp.begin(),
                    interferencesTmp.end());
        }
        InterferencePlanWriter().write(interferences, fullInterferencePlan);
        std::cout << "Full interference plan written to " << fullInterferencePlan
                << std::endl;
    }
    cancelAndDelete(this->_checkTimer);

    delete this->_gsRadiosRadius;
    delete this->_satRadiosRadius;
    delete this->_radiosInterferenceRadius;
    delete this->_kdTreeAdapter;

}

template<typename T>
void ContactPlanCreatingRadioMedium::initContactsMap(contact_map<T> &contacts,
        const inetp::Radio *radio) {
    unsigned int newRadioNodeId =
            radio->getParentModule()->getParentModule()->par("nodeNo");

    // update contacts data structure with new keys
    // add new radio to existing entries
    for (contact_map_entry<T> &x : contacts) {
        x.second.insert(
                contact_map_entry_entry<T>(newRadioNodeId, std::vector<T*>()));
    }
    // add new entry for new radio
    std::map<unsigned int, std::vector<T*>> newEntry;
    for (const inetp::IRadio *iOldRadio : this->radios) {
        const inetp::Radio *oldRadio = omnetpp::check_and_cast<
                const inetp::Radio*>(iOldRadio);
        unsigned int oldRadioNodeId =
                oldRadio->getParentModule()->getParentModule()->par("nodeNo");
        newEntry.insert(
                contact_map_entry_entry<T>(oldRadioNodeId, std::vector<T*>()));
    }
    contacts.insert(contact_map_entry<T>(newRadioNodeId, newEntry));
}

void ContactPlanCreatingRadioMedium::addRadio(const inetp::IRadio *iRadio) {
    const inetp::Radio *radio = omnetpp::check_and_cast<const inetp::Radio*>(
            iRadio);
    // call radio medium function
    inetp::RadioMedium::addRadio(radio);

    this->radios.push_back(iRadio);
    this->initContactsMap(this->_contacts, radio);

    unsigned int newRadioNodeId =
            radio->getParentModule()->getParentModule()->par("nodeNo");

    //add new sink entry at old radios source entries
    for (interference_map_entry<interference_plan_entry_t> &source : this->_interferences) {
        interference_map_inner<interference_plan_entry_t> &map_inner =
                source.second;
        interference_map_entry_entry<interference_plan_entry_t> entry =
                interference_map_entry_entry<interference_plan_entry_t>(
                        newRadioNodeId,
                        interference_map_inner_inner<interference_plan_entry_t>());
        map_inner.insert(entry);
    }
    //add new source entry for new radio
    this->_interferences.insert(
            interference_map_entry<interference_plan_entry_t>(newRadioNodeId,
                    interference_map_inner<interference_plan_entry_t>()));
    for (const unsigned int id : getIRadioIds(this->radios)) {
        this->_interferences[newRadioNodeId].insert(
                interference_map_entry_entry<interference_plan_entry_t>(id,
                        interference_map_inner_inner<interference_plan_entry_t>()));
    }
    // keep track of the radios ourselves
    //this->radios.push_back(radio);
}

//with kd tree
void ContactPlanCreatingRadioMedium::handleMessage(cMessage *message) {
    if (message == this->_checkTimer) {
        inetu::s simtime_limit = parseSimTimeLimit(
                cSimulation::getActiveEnvir()->getConfig()->getConfigValue(
                        "sim-time-limit"));
        if ((simTime() + this->_checkInterval).dbl() <= simtime_limit.get())
            this->scheduleAt(simTime() + this->_checkInterval,
                    this->_checkTimer);
        else if (simTime() != simtime_limit.get())
            this->scheduleAt(simtime_limit.get(), this->_checkTimer);

        //std::cout << "time: " << simTime() << std::endl;
        //creating kd-Tree with all radios
        this->_kdTreeAdapter->createKDRadioTree<long long>(radios);

        std::vector<std::pair<const inetp::Radio*, const inetp::Radio*>> contactsPossible =
                std::vector<std::pair<const inetp::Radio*, const inetp::Radio*>>();


        // check satellite contacts
        if(_checkISL){
            radius_map::iterator it;
            for (it = this->_satRadiosRadius->begin();
                    it != this->_satRadiosRadius->end(); it++) {
                const inetp::IRadio *iTransmitter = it->first;
                const inetp::Radio *transmitter =
                        check_and_cast<const inetp::Radio*>(iTransmitter);
                unsigned int transmitterNodeId =
                        transmitter->getParentModule()->getParentModule()->par(
                                "nodeNo");
                //searching for all satellites that are in the range radius and check them
                std::vector<std::pair<long long, long long> > radiusSearchResult;

                this->_kdTreeAdapter->unsortedRadiusSearch<int>(transmitterNodeId,
                        radiusSearchResult, (long long) it->second);
                for (const std::pair<long long, long long> ID_Radius : radiusSearchResult) {
                    // match Id, as node id starts at 1
                    unsigned int receiverNodeId = ID_Radius.first + 1;

                    //get first radio of satellite with id
                    const NodeRegistry *nodeRegistry = NodeRegistry::getInstance();
                    auto networkHost =
                            nodeRegistry->getNode(receiverNodeId)->getSubmodule(
                                    "networkHost");
                    const IRadio *iReceiver = check_and_cast<const inetp::IRadio*>(
                            networkHost->getSubmodule("wlan", 0)->getSubmodule(
                                    "radio"));
                    const inetp::Radio *receiver = check_and_cast<
                            const inetp::Radio*>(iReceiver);

                    if (iReceiver == iTransmitter) {
                        // receiver and transmitter need to be different
                        continue;
                    }
                    if (0
                            != strcmp(
                                    receiver->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                                    "estnet::Satellite")) {
                        //sat to gs connections will be checked later
                        continue;
                    }
                    if (this->isBidirectionalContact(transmitterNodeId,
                            receiverNodeId) && transmitterNodeId > receiverNodeId) {
                        // if we assume all satellite contacts are birectional
                        // we can skip the reversed order checks.
                        // sat-to-ground or ground-to-sat contacts are usually very
                        // different, so we'll keep doing directional checks there
                        contactsPossible.push_back(
                                std::pair<const inetp::Radio*, const inetp::Radio*>(
                                        transmitter, receiver));
                        continue;
                    }
                    // check if contact possible without interference
                    bool isWorkingContact = this->checkAndAddWorkingContacts(
                            transmitter, receiver);
                    // check who can interfer with the contact, if the contact is possible
                    if (isWorkingContact && this->_buildInterferencePlan) {
                        contactsPossible.push_back(
                                std::pair<const inetp::Radio*, const inetp::Radio*>(
                                        transmitter, receiver));
                    }
                }
            }
        }
        radius_map::iterator it;
        for (it = this->_gsRadiosRadius->begin();
                it != this->_gsRadiosRadius->end(); it++) {
            const inetp::IRadio *iTransmitter = it->first;
            const inetp::Radio *transmitter =
                    check_and_cast<const inetp::Radio*>(iTransmitter);
            unsigned int transmitterNodeId =
                    transmitter->getParentModule()->getParentModule()->par(
                            "nodeNo");
            //searching for all nodes that are in the range radius and check them
            std::vector<std::pair<long long, long long> > radiusSearchResult;
            this->_kdTreeAdapter->unsortedRadiusSearch<int>(transmitterNodeId,
                    radiusSearchResult, (long long) it->second);
            for (const std::pair<long long, long long> ID_Radius : radiusSearchResult) {
                unsigned int receiverNodeId = ID_Radius.first + 1;

                //get first radio of satellite with id
                const NodeRegistry *nodeRegistry = NodeRegistry::getInstance();
                auto networkHost =
                        nodeRegistry->getNode(receiverNodeId)->getSubmodule(
                                "networkHost");
                const IRadio *iReceiver = check_and_cast<const inetp::IRadio*>(
                        networkHost->getSubmodule("wlan", 0)->getSubmodule(
                                "radio"));
                const inetp::Radio *receiver = check_and_cast<
                        const inetp::Radio*>(iReceiver);

                if (iReceiver == iTransmitter) {
                    // receiver and transmitter need to be different
                    continue;
                }

                // check for gs to gs contact
                if (0 != strcmp(receiver->getParentModule()->getParentModule()->getParentModule()->getClassName(),
                                "estnet::Satellite")) {
                    // check if contact gs to gs is possible, no interference check necessary
                    bool isWorkingContact = this->checkAndAddWorkingContacts(
                            transmitter, receiver);
                    bool isWorkingContactOther = this->checkAndAddWorkingContacts(
                            receiver, transmitter);
                    continue;
                }

                // check if contact gs to sat is possible without interference
                if(_checkUplinks){
                    bool isWorkingContact = this->checkAndAddWorkingContacts(
                            transmitter, receiver);
                    // check who can interfere with the contact, if the contact is possible
                    if (isWorkingContact && this->_buildInterferencePlan) {
                        contactsPossible.push_back(
                                std::pair<const inetp::Radio*, const inetp::Radio*>(
                                        transmitter, receiver));
                    }
                }
                if(_checkDownlinks){
                    // check if contact sat to gs is possible without interference
                    bool isWorkingContactOther = this->checkAndAddWorkingContacts(
                            receiver, transmitter);
                    // check who can interfere with the contact, if the contact is possible
                    if (isWorkingContactOther && this->_buildInterferencePlan) {
                        contactsPossible.push_back(
                                std::pair<const inetp::Radio*, const inetp::Radio*>(
                                        receiver, transmitter));
                    }
                }
            }
        }
        std::set<const inetp::Radio*> sourceRadios = std::set<
                const inetp::Radio*>();
        for (auto contact : contactsPossible) {
            sourceRadios.insert(contact.first);
        }
        _sourceIdsInTimeStep.clear();
        _sourceIdsInTimeStep = this->getRadioIds(sourceRadios);
        for (auto contact : contactsPossible) {
            this->checkAndAddInterferingContacts(contact.first, contact.second);
        }
    } else {
        RadioMedium::handleMessage(message);
    }
}

//old version
/*
 void ContactPlanCreatingRadioMedium::handleMessage(cMessage *message) {
 if (message == this->_checkTimer) {
 this->scheduleAt(simTime() + this->_checkInterval, this->_checkTimer);
 this->_kdTreeAdapter->createKDRadioTree<long long>(radios);

 for (const inetp::IRadio *iTransmitter : radios) {
 const inetp::Radio *transmitter =
 check_and_cast<const inetp::Radio *>(iTransmitter);
 unsigned int transmitterNodeId =
 transmitter->getParentModule()->getParentModule()->par("nodeNo");

 for (const inetp::IRadio *iReceiver : radios) {
 const inetp::Radio *receiver =
 check_and_cast<const inetp::Radio *>(iReceiver);
 unsigned int receiverNodeId =
 receiver->getParentModule()->getParentModule()->par("nodeNo");
 if (iReceiver == iTransmitter) {
 // receiver and transmitter need to be different
 continue;
 }
 if (this->isBidirectionalContact(transmitterNodeId, receiverNodeId) && transmitterNodeId > receiverNodeId) {
 // if we assume all satellite contacts are birectional
 // we can skip the reversed order checks.
 // sat-to-ground or ground-to-sat contacts are usually very
 // different, so we'll keep doing directional checks there
 continue;
 }
 // check if contact possible without interference
 bool isWorkingContact = this->checkAndAddWorkingContacts(transmitter, receiver);
 // check who can interfer with the contact, if the contact is possible
 if (isWorkingContact) {
 this->checkAndAddInterferingContacts(transmitter, receiver);
 }
 }
 }
 } else {
 RadioMedium::handleMessage(message);
 }
 }

 */

bool ContactPlanCreatingRadioMedium::isBidirectionalContact(
        unsigned int sourceNodeId, unsigned int sinkNodeId) const {
    const GroundStation *sourceGs =
            NodeRegistry::getInstance()->getGroundStation(sourceNodeId);
    const GroundStation *sinkGs = NodeRegistry::getInstance()->getGroundStation(
            sinkNodeId);
    bool satToSatContact = (sourceGs == nullptr && sinkGs == nullptr);
    return this->_assumeBidirectionalSatContacts && satToSatContact;
}

contact_plan_entry_t* ContactPlanCreatingRadioMedium::buildContact(
        unsigned int sourceNodeId, unsigned int sinkNodeId, int64_t bitrate,
        int64_t range, bool enabled) const {
    // if source and sink are ground stations and can communicate without radio
    // (they are internet connected)
    // then we throw away their contacts for now, as they will be always connected
    // and we generate them later.
    // that way we don't have overlapping entries in the contact plan for the same
    // node pair.
    const GroundStation *sourceGs =
            NodeRegistry::getInstance()->getGroundStation(sourceNodeId);
    const GroundStation *sinkGs = NodeRegistry::getInstance()->getGroundStation(
            sinkNodeId);
    bool sourceIsGs = (sourceGs != nullptr);
    bool sinkIsGs = (sinkGs != nullptr);
    if (sourceIsGs && sinkIsGs
            && sourceGs->canCommunicateWithoutRadioWith(sinkNodeId)) {
        return nullptr;
    }

    contact_plan_entry_t *newContactPlanEntry = new contact_plan_entry_t;
    memset(newContactPlanEntry, 0, sizeof(contact_plan_entry_t));
    newContactPlanEntry->sourceNodeId = sourceNodeId;
    newContactPlanEntry->sinkNodeId = sinkNodeId;
    newContactPlanEntry->sourceIsGroundStation = sourceIsGs;
    newContactPlanEntry->sinkIsGroundStation = sinkIsGs;
    newContactPlanEntry->startTime = omnetpp::simTime().inUnit(
            omnetpp::SIMTIME_S);
    newContactPlanEntry->endTime = omnetpp::simTime().inUnit(
            omnetpp::SIMTIME_S);
    newContactPlanEntry->bitrate = bitrate; // pessimistically rounding down
    newContactPlanEntry->range = range; // rounding up, because the minimum is 1
    newContactPlanEntry->minDistance = 0;
    newContactPlanEntry->maxDistance = 0;
    newContactPlanEntry->enabled = enabled;
    return newContactPlanEntry;
}

void ContactPlanCreatingRadioMedium::addContact(
        contact_map<contact_plan_entry_t> &contacts,
        contact_plan_entry_t *&newContactPlanEntry) {
    std::vector<contact_plan_entry_t*> &nodePairContacts =
            contacts[newContactPlanEntry->sourceNodeId][newContactPlanEntry->sinkNodeId];
    // if there is already a contact and it's also enabled/not_enabled, we can
    // just extends the endTime
    bool addNewEntry = true;
    if (!nodePairContacts.empty()) {
        contact_plan_entry_t *lastEntry = nodePairContacts.back();
        if ((lastEntry->enabled == newContactPlanEntry->enabled)
                && ((newContactPlanEntry->startTime - lastEntry->endTime)
                        <= this->_checkInterval)) {
            addNewEntry = false;
            lastEntry->endTime = newContactPlanEntry->endTime;
            lastEntry->bitrate = std::max(lastEntry->bitrate,
                    newContactPlanEntry->bitrate);
            lastEntry->range = std::max(lastEntry->range,
                    newContactPlanEntry->range);
            lastEntry->minDistance = std::min(lastEntry->minDistance,
                    newContactPlanEntry->minDistance);
            lastEntry->maxDistance = std::max(lastEntry->maxDistance,
                    newContactPlanEntry->maxDistance);
        }
    }
    // otherwise we add a whole new entry
    if (addNewEntry) {
        nodePairContacts.push_back(newContactPlanEntry);
    }
    // if we assume all contacts are bidirectional, we'll add the reverse contact as well
    if (this->isBidirectionalContact(newContactPlanEntry->sourceNodeId,
            newContactPlanEntry->sinkNodeId)
            && newContactPlanEntry->sourceNodeId
                    < newContactPlanEntry->sinkNodeId) {
        contact_plan_entry_t *newContactPlanEntryReversed =
                new contact_plan_entry_t;
        memcpy(newContactPlanEntryReversed, newContactPlanEntry,
                sizeof(contact_plan_entry_t));
        newContactPlanEntryReversed->sourceNodeId =
                newContactPlanEntry->sinkNodeId;
        newContactPlanEntryReversed->sinkNodeId =
                newContactPlanEntry->sourceNodeId;
        newContactPlanEntryReversed->sourceIsGroundStation =
                newContactPlanEntry->sinkIsGroundStation;
        newContactPlanEntryReversed->sinkIsGroundStation =
                newContactPlanEntry->sourceIsGroundStation;
        this->addContact(contacts, newContactPlanEntryReversed);
    }
}

bool ContactPlanCreatingRadioMedium::isWorkingContact(
        const inetp::Radio *transmitter, const inetp::Radio *receiver,
        int64_t &bitrate, int64_t &range) {
    long transmitterNodeId =
            transmitter->getParentModule()->getParentModule()->par("nodeNo");
    long receiverNodeId = receiver->getParentModule()->getParentModule()->par(
            "nodeNo");

    const inetp::IReceptionDecision *receptionDecision = doFakeTransmission(
            transmitter, receiver, std::vector<inetp::Radio*>(), _packetLength);

    EV_INFO << "!!!! ContactPlanCreatingRadioMedium::isWorkingContact from "
                   << transmitterNodeId << " to " << receiverNodeId << ": "
                   << ", possible " << receptionDecision->isReceptionPossible()
                   << ", attempted "
                   << receptionDecision->isReceptionAttempted()
                   << ", successful "
                   << receptionDecision->isReceptionSuccessful()
                   << omnetpp::endl;
    bool result = receptionDecision->isReceptionSuccessful();
    EV_INFO << "!!!! ContactPlanCreatingRadioMedium::isWorkingContact from "
                   << transmitterNodeId << " to " << receiverNodeId << ": "
                   << result << omnetpp::endl;

    auto reception = receptionDecision->getReception();
    auto transmission = reception->getTransmission();
    bitrate = transmission->getPacket()->getBitLength()
            / transmission->getDuration();
    double propagationDuration = (reception->getStartTime()
            - transmission->getStartTime()).dbl();
    range = std::round(propagationDuration + 0.5); // rounding up, because the minimum is 1;

    freeFakeReceptionDecision(receptionDecision);
    return result;
}

interference_plan_entry_t* ContactPlanCreatingRadioMedium::buildInterferenceEntry(
        unsigned int sourceNodeId, unsigned int sinkNodeId,
        std::vector<unsigned int> interferingNodeId, int64_t startTime,
        int64_t endTime, bool enabled) const {
    interference_plan_entry_t *newInterferencePlanEntry =
            new interference_plan_entry_t;
    memset(newInterferencePlanEntry, 0, sizeof(interference_plan_entry_t));
    newInterferencePlanEntry->sourceNodeId = sourceNodeId;
    newInterferencePlanEntry->sinkNodeId = sinkNodeId;
    newInterferencePlanEntry->interferingNodeId = interferingNodeId;
    newInterferencePlanEntry->startTime = startTime;
    newInterferencePlanEntry->endTime = endTime;
    newInterferencePlanEntry->enabled = enabled;
    return newInterferencePlanEntry;
}

void ContactPlanCreatingRadioMedium::addInterferenceEntry(
        interference_map<interference_plan_entry_t> &interferences,
        interference_plan_entry_t *newInterferencePlanEntry) {
    if (interferences[newInterferencePlanEntry->sourceNodeId][newInterferencePlanEntry->sinkNodeId].count(
            newInterferencePlanEntry->interferingNodeId) == 0) {
        interferences[newInterferencePlanEntry->sourceNodeId][newInterferencePlanEntry->sinkNodeId].insert(
                interference_map_entry_entry_entry<interference_plan_entry_t>(
                        newInterferencePlanEntry->interferingNodeId,
                        InterferencePlan()));
    }
    InterferencePlan &nodePairContacts =
            interferences[newInterferencePlanEntry->sourceNodeId][newInterferencePlanEntry->sinkNodeId][newInterferencePlanEntry->interferingNodeId];
    // if there is already a contact and it's endTime is same as startTime of the new, we can
    // just extends the endTime
    bool addNewEntry = true;
    if (!nodePairContacts.empty()) {
        interference_plan_entry_t *lastEntry = nodePairContacts.back();
        if (lastEntry->endTime >= newInterferencePlanEntry->startTime) {
            addNewEntry = false;
            lastEntry->endTime = newInterferencePlanEntry->endTime;
        }
    }
    // otherwise we add a whole new entry
    if (addNewEntry) {
        nodePairContacts.push_back(newInterferencePlanEntry);
    }
    // if we assume all contacts are bidirectional, we'll add the reverse interference as well
    if (this->isBidirectionalContact(newInterferencePlanEntry->sourceNodeId,
            newInterferencePlanEntry->sinkNodeId)
            && newInterferencePlanEntry->sourceNodeId
                    < newInterferencePlanEntry->sinkNodeId) {
        interference_plan_entry_t *newInterferencePlanEntryReversed =
                new interference_plan_entry_t;
        memcpy(newInterferencePlanEntryReversed, newInterferencePlanEntry,
                sizeof(interference_plan_entry_t));
        newInterferencePlanEntryReversed->sourceNodeId =
                newInterferencePlanEntry->sinkNodeId;
        newInterferencePlanEntryReversed->sinkNodeId =
                newInterferencePlanEntry->sourceNodeId;
        this->addInterferenceEntry(interferences,
                newInterferencePlanEntryReversed);
    }


}


const IArrival *ContactPlanCreatingRadioMedium::getArrival(const IRadio *receiver, const ITransmission *transmission) const
{
    return this->propagation->computeArrival(transmission,
                            receiver->getAntenna()->getMobility());
}


const IReception *ContactPlanCreatingRadioMedium::getReception(const IRadio *receiver, const ITransmission *transmission) const
{
    const IReception *reception = computeReception(receiver, transmission);
    return reception;
}

const IListening *ContactPlanCreatingRadioMedium::getListening(const IRadio *receiver, const ITransmission *transmission) const
{
    return receiver->getReceiver()->createListening(receiver,
            transmission->getStartTime(), transmission->getEndTime(),
            transmission->getStartPosition(),
            transmission->getEndPosition());
}

const inetp::IReceptionDecision* ContactPlanCreatingRadioMedium::doFakeTransmission(
        const inetp::Radio *transmitter, const inetp::Radio *receiver,
        const std::vector<inetp::Radio*> &interferences,
        int64_t packetBitLength) {

    // create fake packet for transmission with same size
    inet::Packet *fakePacket = new inet::Packet("fakePacket");
    auto data = makeShared<BitCountChunk>(b(packetBitLength));
    //Chunk::enableImplicitChunkSerialization = true;
    fakePacket->insertAtBack(data);

    auto packetProtocolTag =
            fakePacket->addTagIfAbsent<inet::PacketProtocolTag>();
    packetProtocolTag->setProtocol(&inet::Protocol::ipv4);

    transmitter->encapsulate(fakePacket);

    // create fake transmission
    const inetp::ITransmission *fakeTransmission =
            transmitter->getTransmitter()->createTransmission(transmitter,
                    fakePacket, omnetpp::simTime());
    // create fake arrival
    const inetp::IArrival *fakeArrival = this->propagation->computeArrival(
            fakeTransmission, receiver->getAntenna()->getMobility());


    // build interfering receptions
    std::vector<const inetp::IReception*> interferingReceptionsTmp;
    for (const auto &interferingTransmitterRadio : interferences) {
        // calculate when interferring transmission needs to occur to arrive at the same time
        // assumes interferingTransmitterRadio and receiver use the same bitrate
        omnetpp::simtime_t startArrivalTime = fakeArrival->getStartTime();
        inet::mps propagationSpeed = inet::mps(
                omnetpp::check_and_cast<const inetp::PropagationBase*>(
                        this->propagation)->par("propagationSpeed"));
        const inet::Coord interferingTransmitterPos =
                interferingTransmitterRadio->getAntenna()->getMobility()->getCurrentPosition();
        const inet::Coord receiverPos = fakeArrival->getStartPosition();
        omnetpp::simtime_t propagationTime = receiverPos.distance(
                interferingTransmitterPos) / propagationSpeed.get();
        omnetpp::simtime_t transmitTime = startArrivalTime - propagationTime;

        // create fake packet for transmission with same size
        inet::Packet *interferingPacket = new inet::Packet("interferingPacket");
        auto dataInt = makeShared<BitCountChunk>(b(packetBitLength));
        interferingPacket->insertAtBack(dataInt);

        auto packetProtocolTag = interferingPacket->addTagIfAbsent<
                inet::PacketProtocolTag>();
        packetProtocolTag->setProtocol(&inet::Protocol::ipv4);
        interferingTransmitterRadio->encapsulate(interferingPacket);

        // create fake transmission
        const inetp::ITransmission *interferingTransmission =
                interferingTransmitterRadio->getTransmitter()->createTransmission(
                        interferingTransmitterRadio, interferingPacket,
                        transmitTime);
        // create fake arrival
        const inetp::IArrival *interferingArrival =
                this->propagation->computeArrival(interferingTransmission,
                        receiver->getAntenna()->getMobility());
        // crate fake reception
        const inetp::IReception *interferingReception =
                this->analogModel->computeReception(receiver,
                        interferingTransmission, interferingArrival);
        //this->addTransmission(interferingTransmitterRadio,interferingTransmission);

        EV_DEBUG
                        << "ContactPlanCreatingRadioMedium::doFakeTransmission receptionStartTime "
                        << fakeArrival->getStartTime()
                        << ", interferingReceptionStartTime "
                        << interferingReception->getStartTime()
                        << omnetpp::endl;
        // put into our vector of interferences
        interferingReceptionsTmp.push_back(interferingReception);
        // arrival is not needed anymore and will fall out of scope => freeing
        delete interferingArrival;
    }
    // recreate fake arrival an transmittsion to have larger transmission id than interferences
    // create fake transmission
    auto fakeTransmission2 = transmitter->getTransmitter()->createTransmission(transmitter,
                    fakePacket, omnetpp::simTime());
    //this->addTransmission(transmitter,fakeTransmission2);
    // create fake arrival
    auto fakeArrival2 = this->propagation->computeArrival(
            fakeTransmission2, receiver->getAntenna()->getMobility());

    const inetp::IListening *fakeListening =
             receiver->getReceiver()->createListening(receiver,
                     fakeArrival->getStartTime(), fakeArrival->getEndTime(),
                     fakeArrival->getStartPosition(),
                     fakeArrival->getEndPosition());

    // create fake reception
    const inetp::IReception *fakeReception =
            this->analogModel->computeReception(receiver, fakeTransmission2,
                    fakeArrival2);
    const inetp::INoise *bNoise =
            backgroundNoise ?
                    backgroundNoise->computeNoise(fakeListening) : nullptr;
    // check reception decision with interference
    inetp::IInterference *interferenceTmp = new inetp::Interference(bNoise,
            &interferingReceptionsTmp);
    const inetp::INoise *noiseTmp = this->analogModel->computeNoise(
            fakeListening, interferenceTmp);
    const inetp::ISnir *snirTmp = this->analogModel->computeSNIR(fakeReception,
            noiseTmp);
    // check if the original reception is interfered by our fake reception
    const inetp::IReceptionDecision *receptionDecisionTmp =
            receiver->getReceiver()->computeReceptionDecision(fakeListening,
                    fakeReception, inetp::IRadioSignal::SIGNAL_PART_WHOLE,
                    interferenceTmp, snirTmp);
    //this->removeTransmission(fakeTransmission2);


    // free all our memory
    delete snirTmp;
    delete noiseTmp;
    for (const auto &interferingReception : interferingReceptionsTmp) {
        auto interferingTransmission = interferingReception->getTransmission();
        auto interferingPacket = interferingTransmission->getPacket();
        //this->removeTransmission(interferingTransmission);

        delete interferingTransmission;
        delete interferingPacket;
        delete interferingReception;
    }
    if (bNoise != nullptr) {
        delete bNoise;
    }
    delete fakeListening;
    delete fakeArrival;

    return receptionDecisionTmp;
}

void ContactPlanCreatingRadioMedium::freeFakeReceptionDecision(
        const inetp::IReceptionDecision *receptionDecision) const {
    delete receptionDecision->getReception()->getTransmission()->getPacket();
    delete receptionDecision->getReception()->getTransmission();
    delete receptionDecision->getReception();
    delete receptionDecision;
}

bool ContactPlanCreatingRadioMedium::checkAndAddWorkingContacts(
        const inetp::Radio *transmitter, const inetp::Radio *receiver) {
    int64_t bitrate, range;
    unsigned int transmitterNodeId =
            transmitter->getParentModule()->getParentModule()->par("nodeNo");
    unsigned int receiverNodeId =
            receiver->getParentModule()->getParentModule()->par("nodeNo");

    bool isWorkingContact = this->isWorkingContact(transmitter, receiver,
            bitrate, range);
    contact_plan_entry_t *newContact = this->buildContact(transmitterNodeId,
            receiverNodeId, bitrate, range, isWorkingContact);
    if (newContact != nullptr) {
        this->addContact(this->_contacts, newContact);
    }
    return isWorkingContact;
}

void ContactPlanCreatingRadioMedium::checkAndAddInterferingContacts(
        const inetp::Radio *transmitter, const inetp::Radio *receiver) {
    /*
     * right now we're checking which transmitter
     * disturbs a particular receiver.
     * With directional antennas, this is pessimistic, because
     * a transmitter only disturbs what is in his RF beam.
     * There we could check which receivers a particular transmitter
     * disturbs, when focusing on one receiver. That is a more effort
     * as it required a double loop over all radios.
     */

    unsigned int transmitterNodeId =
            transmitter->getParentModule()->getParentModule()->par("nodeNo");
    unsigned int receiverNodeId =
            receiver->getParentModule()->getParentModule()->par("nodeNo");
    // skipping interference checks between ground stations
    NodeRegistry *nodeRegistry = NodeRegistry::getInstance();
    if (nodeRegistry->isGroundStation(transmitterNodeId)
            && nodeRegistry->isGroundStation(receiverNodeId)) {
        return;
    }
    inetu::s simtime_limit = parseSimTimeLimit(
            cSimulation::getActiveEnvir()->getConfig()->getConfigValue(
                    "sim-time-limit"));
    int64_t startTime = simTime().inUnit(SIMTIME_S) - this->_checkInterval;
    if (startTime < 0)
        startTime = 0;
    int64_t endTime = simTime().inUnit(SIMTIME_S) + this->_checkInterval;
    if (endTime > simtime_limit.get())
        endTime = simtime_limit.get();

    const inet::Coord transmitterPos =
            transmitter->getAntenna()->getMobility()->getCurrentPosition();
    const inet::Coord receiverPos =
            receiver->getAntenna()->getMobility()->getCurrentPosition();

    std::vector<Radio*> possibleAddedInterferer;
    std::vector<const inetp::IRadio*> possibleInterferer;

    //do radius search for possible interferer 2x if sink sat, otherwise 1x

    long long radius = this->_radiosInterferenceRadius->at(receiver);
    std::vector<std::pair<long long, long long>> radiusSearchResult;
    this->_kdTreeAdapter->unsortedRadiusSearch<int>(receiverNodeId,
            radiusSearchResult, radius);
    for (const std::pair<int64_t, int64_t> ID_Radius : radiusSearchResult) {
        unsigned int interfererNodeId = ID_Radius.first + 1;
        //check if this node has possible contact at all
        if (std::find(this->_sourceIdsInTimeStep.begin(),
                this->_sourceIdsInTimeStep.end(), interfererNodeId)
                == this->_sourceIdsInTimeStep.end()) {
            continue;
        }
        //get first radio of satellite with interfererNodeId
        auto networkHost =
                nodeRegistry->getNode(interfererNodeId)->getSubmodule(
                        "networkHost");
        const IRadio *iInterferer = check_and_cast<const inetp::IRadio*>(
                networkHost->getSubmodule("wlan", 0)->getSubmodule("radio"));
        possibleInterferer.push_back(iInterferer);
    }

    //not using radius search optimization
    //possibleInterferer = std::vector<const inetp::IRadio*>(radios);

    // going through all possible interferer to check if they could interfer with our receiver right now
    for (const inetp::IRadio *iRadioTransmitter : possibleInterferer) {
        std::vector<unsigned int> interferingTransmitterNodeId;
        const inetp::Radio *interferingTransmitterRadio = check_and_cast<
                const inetp::Radio*>(iRadioTransmitter);
        inetp::Radio *interferingTransmitterRadio2 =
                const_cast<inetp::Radio*>(interferingTransmitterRadio);
        // skipping the transmitter if it is our receiver
        if (interferingTransmitterRadio == receiver) {
            continue;
        }
        // skipping the transmitter if it is our original transmitter
        if (interferingTransmitterRadio == transmitter) {
            continue;
        }
        interferingTransmitterNodeId.push_back(
                interferingTransmitterRadio->getParentModule()->getParentModule()->par(
                        "nodeNo"));
        // skipping if interfering transmitter and receiver are both ground stations
        if (nodeRegistry->isGroundStation(interferingTransmitterNodeId.at(0))
                && nodeRegistry->isGroundStation(receiverNodeId)) {
            continue;
        }
        if (!this->_considerAddedInterferences) { //check single interferer
            const inet::Coord interfererPos =
                    interferingTransmitterRadio->getAntenna()->getMobility()->getCurrentPosition();
            const double distanceInterferToTransmitter = interfererPos.distance(
                    transmitterPos);
            const double distanceInterferToReceiver = interfererPos.distance(
                    receiverPos);

            EV_INFO
                           << "!!!! ContactPlanCreatingRadioMedium::hasInterferingContacts "
                           << "checking interference for contact "
                           << transmitterNodeId << " -> " << receiverNodeId
                           << " from transmitter "
                           //<< interferingTransmitterNodeId
                           << omnetpp::endl;

            // check if the original reception is interfered by our fake reception
            std::vector<inetp::Radio*> interferingTransmitters;
            interferingTransmitters.push_back(interferingTransmitterRadio2);
            const inetp::IReceptionDecision *receptionDecisionTmp;

            if (!this->isBidirectionalContact(transmitterNodeId, receiverNodeId)
                    || distanceInterferToTransmitter
                            > distanceInterferToReceiver) {
                receptionDecisionTmp = doFakeTransmission(transmitter, receiver,
                        interferingTransmitters, 800);
            } else {
                receptionDecisionTmp = doFakeTransmission(receiver, transmitter,
                        interferingTransmitters, 800);
            }

            EV_INFO
                           << "!!!! ContactPlanCreatingRadioMedium::hasInterferingContacts for "
                           << transmitterNodeId << " -> " << receiverNodeId
                           << " with interference from "
                           << interferingTransmitterNodeId.at(0) << ": "
                           << ", possible "
                           << receptionDecisionTmp->isReceptionPossible()
                           << ", attempted "
                           << receptionDecisionTmp->isReceptionAttempted()
                           << ", successful "
                           << receptionDecisionTmp->isReceptionSuccessful()
                           << endl;

            bool interfered = receptionDecisionTmp->isReceptionPossible()
                    && !receptionDecisionTmp->isReceptionSuccessful();
            if (interfered) {
                interference_plan_entry_t *newInterferencePlanEntry =
                        this->buildInterferenceEntry(transmitterNodeId,
                                receiverNodeId,
                                getRadioIds(interferingTransmitters), startTime,
                                endTime, interfered);
                this->addInterferenceEntry(this->_interferences,
                        newInterferencePlanEntry);
            }
            freeFakeReceptionDecision(receptionDecisionTmp);
        } else if (this->_considerAddedInterferences) {
            //create list of Radio elements for check, in order to save cast using iRadio
            possibleAddedInterferer.push_back(interferingTransmitterRadio2);
        }

    }

    //check added interferer
    if (this->_considerAddedInterferences) {
        std::vector<inetp::Radio*> initVectorCheckedTransmitters;
        std::vector<std::vector<Radio*>> initVectorInterferingCombinations;
        this->furtherInterferenceChecks(transmitter, receiver, startTime,
                endTime, initVectorCheckedTransmitters, possibleAddedInterferer,
                initVectorInterferingCombinations);
    }
}

std::vector<unsigned int> ContactPlanCreatingRadioMedium::getRadioIds(
        std::vector<inetp::Radio*> radios) {
    std::vector<unsigned int> ids;
    //for each radio we add id to return vector
    for (inetp::Radio *radio : radios) {
        ids.push_back(
                radio->getParentModule()->getParentModule()->par("nodeNo"));
    }
    return ids;
}
std::vector<unsigned int> ContactPlanCreatingRadioMedium::getRadioIds(
        std::vector<const inetp::Radio*> radios) {
    std::vector<unsigned int> ids;
    //for each radio we add id to return vector
    for (const inetp::Radio *radio : radios) {
        ids.push_back(
                radio->getParentModule()->getParentModule()->par("nodeNo"));
    }
    return ids;
}

std::vector<unsigned int> ContactPlanCreatingRadioMedium::getRadioIds(
        std::set<const inetp::Radio*> radios) {
    std::vector<unsigned int> ids;
    //for each radio we add id to return vector
    for (const inetp::Radio *radio : radios) {
        ids.push_back(
                radio->getParentModule()->getParentModule()->par("nodeNo"));
    }
    return ids;
}

std::vector<unsigned int> ContactPlanCreatingRadioMedium::getIRadioIds(
        std::vector<const IRadio*> IRadios) {
    std::vector<unsigned int> ids;
    //for each radio we add id to return vector
    for (const inetp::IRadio *iRadio : IRadios) {
        const inetp::Radio *radio = check_and_cast<const inetp::Radio*>(iRadio);
        ids.push_back(
                radio->getParentModule()->getParentModule()->par("nodeNo"));
    }
    return ids;
}

bool ContactPlanCreatingRadioMedium::checkInterfering(
        const inetp::Radio *transmitter, const inetp::Radio *receiver,
        std::vector<inetp::Radio*> interferingTransmitters) {
    const inetp::IReceptionDecision *receptionDecisionTmp;
    //doing fake transmission for checking if the contact is interfered
    receptionDecisionTmp = doFakeTransmission(transmitter, receiver,
            interferingTransmitters, 800);
    bool interfered = receptionDecisionTmp->isReceptionPossible()
            && !receptionDecisionTmp->isReceptionSuccessful();
    EV_INFO << "******* ContactPlanCreatingRadioMedium::checkInterfering for "
                   << transmitter->getParentModule()->getParentModule()->par(
                           "nodeNo").intValue() << " -> "
                   << receiver->getParentModule()->getParentModule()->par(
                           "nodeNo").intValue() << " with interference from ";
    for (auto id : getRadioIds(interferingTransmitters)) {
        EV_INFO << id << ", ";
    }
    EV_INFO << " possible " << receptionDecisionTmp->isReceptionPossible()
                   << ", attempted "
                   << receptionDecisionTmp->isReceptionAttempted()
                   << ", successful "
                   << receptionDecisionTmp->isReceptionSuccessful() << endl;

    freeFakeReceptionDecision(receptionDecisionTmp);
    return interfered;
}

void ContactPlanCreatingRadioMedium::furtherInterferenceChecks(
        const inetp::Radio *transmitter, const inetp::Radio *receiver,
        int64_t startTime, int64_t endTime,
        std::vector<Radio*> alreadyConsideredInterferer,
        std::vector<Radio*> possibleInterferer,
        std::vector<std::vector<Radio*>> &interferenceCombinations) {

    //first we get the ids of the sender
    unsigned int transmitterNodeId =
            transmitter->getParentModule()->getParentModule()->par("nodeNo");
    unsigned int receiverNodeId =
            receiver->getParentModule()->getParentModule()->par("nodeNo");

    //iteration over all possible interferer while deleting the element from the vector after iteration step
    auto itOuter = possibleInterferer.begin();
    std::map<std::vector<Radio*>, std::vector<Radio*>> notInterferingCombination; // considered -> notconsidered

    while (!possibleInterferer.empty()) {
        //for each possible interferer create new combination by adding it to already considered
        auto notConsideredRadioIdsCopy = std::vector<Radio*>(
                possibleInterferer);
        auto consideredRadioIdsCopy = std::vector<Radio*>(
                alreadyConsideredInterferer);
        auto it = notConsideredRadioIdsCopy.begin();
        Radio *id = *it.base();
        notConsideredRadioIdsCopy.erase(it);
        consideredRadioIdsCopy.push_back(id);

        //check if created combination is contained in already found interfering combinations
        bool hasAllRadiosOfCombination = false;
        for (auto iCombination : interferenceCombinations) {
            hasAllRadiosOfCombination = true;
            for (const Radio *iRadioOfCombi : iCombination) {
                bool containsIRadio = false;
                for (const Radio *radio : consideredRadioIdsCopy) {
                    if (radio == iRadioOfCombi) {
                        containsIRadio = true;
                        break;
                    }
                }
                if (!containsIRadio) {
                    hasAllRadiosOfCombination = false;
                    break;
                }
            }
            if (hasAllRadiosOfCombination)
                break;
        }
        if (hasAllRadiosOfCombination) {
            possibleInterferer.erase(itOuter);
            continue;
        }
        //doing interference check
        bool interfered = checkInterfering(transmitter, receiver,
                consideredRadioIdsCopy);
        if (interfered) {
            //check if already found combinations are contained in created combination
            auto combinationInterator = interferenceCombinations.begin();
            for (auto iCombination : interferenceCombinations) {
                bool hasAllRadiosOfCombination = true;
                for (const Radio *newCombinationRadio : consideredRadioIdsCopy) {
                    bool containsIRadio = false;
                    for (const Radio *radio : iCombination) {
                        if (radio == newCombinationRadio) {
                            containsIRadio = true;
                            break;
                        }
                    }
                    if (!containsIRadio) {
                        hasAllRadiosOfCombination = false;
                        break;
                    }
                }
                if (hasAllRadiosOfCombination) {
                    interferenceCombinations.erase(combinationInterator);
                    std::vector<unsigned int> interfererIds = getRadioIds(
                            iCombination);
                    InterferencePlan lastEntry =
                            this->_interferences[transmitterNodeId][receiverNodeId][interfererIds];
                    lastEntry.back()->endTime = startTime;
                } else {
                    combinationInterator++;
                }
            }
            //add interference as new entry
            interference_plan_entry_t *newInterferencePlanEntry =
                    this->buildInterferenceEntry(transmitterNodeId,
                            receiverNodeId, getRadioIds(consideredRadioIdsCopy),
                            startTime, endTime, true);
            this->addInterferenceEntry(this->_interferences,
                    newInterferencePlanEntry);
            interferenceCombinations.push_back(consideredRadioIdsCopy);

        } else { // saving not interfering combination for recursive check later on
            notInterferingCombination.insert(
                    std::pair<std::vector<Radio*>, std::vector<Radio*>>(
                            consideredRadioIdsCopy, notConsideredRadioIdsCopy));
        }
        possibleInterferer.erase(itOuter);
    }
    //going on recursive with all not interfering combinations
    for (auto combi : notInterferingCombination) {
        furtherInterferenceChecks(transmitter, receiver, startTime, endTime,
                combi.first, combi.second, interferenceCombinations);
    }
}

}  // namespace estnet
