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

#ifndef __CONTACTPLANS_LOGGINGRADIOMEDIUM_H__
#define __CONTACTPLANS_LOGGINGRADIOMEDIUM_H__

#include <inet/physicallayer/common/packetlevel/RadioMedium.h>
#include <inet/physicallayer/contract/packetlevel/IRadioMedium.h>
#include <inet/common/packet/Packet.h>

#include "estnet/common/ESTNETDefs.h"
#include "common/KDTreeAdapter.h"
#include "common/Contacts.h"
#include "estnet/common/queue/PQueue.h"

namespace estnet {

// namespace & type shortcuts for commonly used types
namespace inetp = inet::physicallayer;
template<typename T>
using contact_map_inner = std::map<unsigned int,
std::vector<T *>>;
template<typename T>
using contact_map = std::map<unsigned int,
contact_map_inner<T>>;
template<typename T>
using contact_map_entry = std::pair<
const unsigned int,
std::map<unsigned int, std::vector<T *>>>;
template<typename T>
using contact_map_entry_entry = std::pair<
const unsigned int,
std::vector<T *>>;
template<typename T>
using interference_map_inner_inner = std::map<
const std::vector<unsigned int>,
std::vector<T *>>;
template<typename T>
using interference_map_inner = std::map<unsigned int,
interference_map_inner_inner<T>>;
template<typename T>
using interference_map = std::map<unsigned int,
interference_map_inner<T>>;
template<typename T>
using interference_map_entry = std::pair<
const unsigned int,
interference_map_inner<T>>;
template<typename T>
using interference_map_entry_entry = std::pair<
const unsigned int,
interference_map_inner_inner<T>>;
template<typename T>
using interference_map_entry_entry_entry = std::pair<
const std::vector<unsigned int>,
std::vector<T *>>;

using radius_map = std::map<const inetp::IRadio* , int64_t>;

/**
 * Wrapper around a real radio medium implementation
 * which periodically checks for contacts and interferences.
 * At the end of the simulation the resulting contact &
 * Interference plan is written to files.
 */
class ESTNET_API ContactPlanCreatingRadioMedium: public inetp::RadioMedium {
private:
    int _checkInterval;
    bool _assumeBidirectionalSatContacts;
    bool _considerAddedInterferences;
    bool _buildInterferencePlan;

    const int _packetLength = 800;
    KDTreeAdapter *_kdTreeAdapter;  // kd tree that is storing nodes
    RadioMedium *_realRadioMedium;
    radius_map *_satRadiosRadius; // radius for radius search having a satellite
    radius_map *_gsRadiosRadius; // radius for radius search having a groundstation
    radius_map *_radiosInterferenceRadius; // radius for radius search on interference
    std::vector<unsigned int> _sourceIdsInTimeStep;
    std::vector<const IRadio*> radios;
    omnetpp::cMessage *_checkTimer;
    /*
     * map of contacts during simulation for efficient lookup
     * map<source, map<sink, vector<contact>>>
     */
    contact_map<contact_plan_entry_t> _contacts;
    /*
     * map of interferences during simulation for efficient lookup
     * map<source, map<sink, map<interferer, vector<interference>>>>
     */
    interference_map<interference_plan_entry_t> _interferences;

    /* checks whether the contact can be considered bidirectional */
    bool isBidirectionalContact(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;
    /* checks whether transmitter and receiver are in
     * communication range and passes the bitrate and range out */
    bool isWorkingContact(const inetp::Radio *transmitter,
            const inetp::Radio *receiver, int64_t &bitrate,
            int64_t &range) const;

    /* initializes the map saving contacts during the simulation */
    template<typename T>
    void initContactsMap(contact_map<T> &contacts, const inetp::Radio *radio);
    /* checks whether transmitter and receiver in communication range
     * and adds the contact to the contact map */
    bool checkAndAddWorkingContacts(const inetp::Radio *transmitter,
            const inetp::Radio *receiver);
    /* checks for interferences for the contact between transmitter and receiver
     * and adds it to the interference map */
    void checkAndAddInterferingContacts(const inetp::Radio *transmitter,
            const inetp::Radio *receiver);

    /* build a contact plan entry */
    contact_plan_entry_t* buildContact(unsigned int sourceNodeId,
            unsigned int sinkNodeId, int64_t bitrate, int64_t range,
            bool enabled) const;
    /* adds a contact plan entry to the contacts map */
    void addContact(contact_map<contact_plan_entry_t> &contacts,
            contact_plan_entry_t *&newContactPlanEntry);
    /* build an interference plan entry */
    interference_plan_entry_t* buildInterferenceEntry(unsigned int sourceNodeId,
            unsigned int sinkNodeId,
            std::vector<unsigned int> interferingNodeId, int64_t startTime,
            int64_t endTime, bool enabled) const;
    /* adds an interference plan entry to the interferences map */
    void addInterferenceEntry(
            interference_map<interference_plan_entry_t> &interferences,
            interference_plan_entry_t *newInterferencePlanEntry);

    /* converts the contact map to a list of contacts */
    template<typename T>
    std::vector<T*> getVectorOfContacts(const contact_map<T> &contacts) const;
    template<typename T>
    std::vector<T*> getVectorOfInterferingContacts(
            const interference_map_inner<T> &contacts) const;
    /* adds loopback contacts to the given contact list */
    void addLoopbackContacts(std::vector<contact_plan_entry_t*> &contacts,
            int64_t totalEndTime);
    /* adds internet connected ground stations to the given contact list */
    void addInternetGroundStationContacts(
            std::vector<contact_plan_entry_t*> &contacts, int64_t totalEndTime);

    /* inverts the layout of the interference map to save it to a file */
    interference_map_inner_inner<interference_plan_entry_t> invertInterferenceMap(
            interference_map<interference_plan_entry_t> &interferences) const;

    /* performs a fake transmission and returns the reception decision */
    const inetp::IReceptionDecision* doFakeTransmission(
            const inetp::Radio *transmitter, const inetp::Radio *receiver,
            const std::vector<inetp::Radio*> &interferences,
            int64_t packetBitLength) const;
    /* frees a reception decision */
    void freeFakeReceptionDecision(
            const inetp::IReceptionDecision *receptionDecision) const;

    /* checks if contact is interfered by interferer*/
    bool checkInterfering(const inetp::Radio *transmitter,
            const inetp::Radio *receiver,
            std::vector<inetp::Radio*> interferingTransmitters);
    /* recursive method for checking all combination of added interferer*/
    void furtherInterferenceChecks(const inetp::Radio *transmitter,
            const inetp::Radio *receiver, int64_t startTime, int64_t endTime,
            std::vector<Radio*> alreadyConsideredInterferer,
            std::vector<Radio*> possibleInterferer,
            std::vector<std::vector<Radio*>> &interferenceCombinations);

    /* changes radio vector to id vector*/
    std::vector<unsigned int> getRadioIds(std::vector<inetp::Radio*> radios);
    /* changes radio vector to id vector*/
    std::vector<unsigned int> getRadioIds(
            std::vector<const inetp::Radio*> radios);
    /* changes radio vector to id vector*/
    std::vector<unsigned int> getRadioIds(std::set<const inetp::Radio*> radios);
    /* changes iRadio vector to id vector*/
    std::vector<unsigned int> getIRadioIds(std::vector<const IRadio*> IRadios);

protected:
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief cleanup & plan writing */
    virtual void finish() override;
    /** @brief scheduled self-messages receiver function */
    virtual void handleMessage(omnetpp::cMessage *message) override;

public:

    virtual void addRadio(const inetp::IRadio *iRadio) override;

};
}  // namespace estnet

#endif
