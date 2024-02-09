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

#ifndef __CONTACTPLANS_MANAGER_H__
#define __CONTACTPLANS_MANAGER_H__

#include "Contacts.h"

#include <algorithm> // push_heap, pop_heap
#include <vector> // vector

#include "../iterator/ContactPlanIterator.h"
#include "Semaphore.h"

namespace estnet {

/**
 * Reads a contact plan and provides
 * all modules in the simulation
 * access to all or just active contacts.
 */
class ESTNET_API ContactPlanManager: public omnetpp::cSimpleModule {
private:
    omnetpp::cMessage *_selfMsg;
    ContactPlanIterator<contact_plan_entry_t> _cpIterator;
    Semaphore _semaphore;
    omnetpp::simsignal_t connectionSignalId;
    omnetpp::simsignal_t satSignalId;
    omnetpp::simsignal_t simTimeSignalId;

protected:
    /** @brief returns number of initalization stages */
    virtual int numInitStages() const override;
    /** @brief initialization */
    virtual void initialize(int stage) override;
    /** @brief self message receiver function */
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    /** @brief cleanup function */
    virtual void finish() override;

public:
    /** @brief should only be invoked by the simulation, will throw exception afterwards */
    ContactPlanManager();
    /** @brief cleanup */
    virtual ~ContactPlanManager();
    /** @brief returns the singleton instance of the class */
    static ContactPlanManager* getInstance();

    /*
     * returns all contacts
     */
    const std::vector<contact_plan_entry_t*> getAllContacts() const;

    /*
     * returns all currently active contacts
     */
    const std::vector<contact_plan_entry_t*> getActiveContacts() const;

    /*
     * returns the currently active contact from sourceNodeId to sinkNodeId
     * returns nullptr if none exists
     */
    const contact_plan_entry_t*
    getActiveContactFor(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;

    /*
     * returns the currently active contact from sourceNodeId to sinkNodeId
     * returns nullptr if none exists
     */
    const contact_plan_entry_t*
    getActiveContactFor(unsigned int sourceNodeId) const;

    /*
     * returns true, if there is a currently active contact from sourceNodeId to
     * sinkNodeId
     * returns false otherwise
     */
    bool hasActiveContactFor(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;
    /*
     * returns an upcoming contact from sourceNodeId to sinkNodeId
     * returns nullptr if none exists
     */
    const contact_plan_entry_t*
    getUpcomingContactFor(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;

    /*
     * returns an upcoming contact for sourceNodeId
     * returns nullptr if none exists
     */
    const contact_plan_entry_t*
    getUpcomingContactFor(unsigned int sourceNodeId) const;
    /*
     * returns true, if there is an upcoming contact from sourceNodeId to
     * sinkNodeId in the future
     * returns false otherwise
     */
    bool hasUpcomingContactFor(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;
    /*
     * returns the currently active or an upcoming contact from sourceNodeId to
     * sinkNodeId
     * returns nullptr if none exists
     */
    const contact_plan_entry_t* getNextContactFor(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;

    /*
     * returns the currently active or an upcoming contact from sourceNodeId any
     * sinkNodeId
     * returns nullptr if none exists
     */
    const contact_plan_entry_t* getNextContactFor(
            unsigned int sourceNodeId) const;

    /*
     * returns true, if there is a currently active or upcoming contact from
     * sourceNodeId to sinkNodeId
     * returns false otherwise
     */
    bool hasNextContactFor(unsigned int sourceNodeId,
            unsigned int sinkNodeId) const;

    /*
     * called by protocols to notify manager
     * that their limbo was flushed
     * @deprecated
     */
    void limboFlushedNotification(unsigned int nodeId);

    /*
     * returns the number of seconds this node has had incoming connections from any other node.
     */
    int getDownlinkCoverageForNode(unsigned int nodeId);

    /*
     * returns the number of seconds this node a connection to any other node.
     */
    int getUplinkCoverageForNode(unsigned int nodeId);
};

}  // namespace estnet

#endif
