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

#include "ContactPlanManager.h"
#include "../reader/ContactPlanReader.h"

#include <iostream>
#include <fstream>

#include "estnet/common/node/NodeRegistry.h"

namespace estnet {

static ContactPlanManager *instance;

Define_Module(ContactPlanManager);

extern "C" {
extern void shouldFlushLimbo(int value);
}

ContactPlanManager::ContactPlanManager() {
    if (instance != nullptr) {
        throw omnetpp::cRuntimeError(
                "There can be only one ContactPlanManager instance in the network");
    }
    instance = this;
}

ContactPlanManager::~ContactPlanManager() {
    this->cancelAndDelete(this->_selfMsg);
    instance = nullptr;
}

ContactPlanManager* ContactPlanManager::getInstance() {
    if (instance == nullptr) {
        throw omnetpp::cRuntimeError(
                "ContactPlanManager::getInstance(): there is no "
                        "ContactPlanManager module in the network");
    }
    return instance;
}

int ContactPlanManager::numInitStages() const {
    return 3;
}

void ContactPlanManager::initialize(int stage) {
    if (stage == 0) {
        this->_selfMsg = new omnetpp::cMessage("contactPlanChange");
        std::string contactPlanFilePath =
                this->par("contactPlanFile").stringValue();

        std::vector<contact_plan_entry_t*> contacts = ContactPlanReader().read(
                contactPlanFilePath);
        this->_cpIterator.setContacts(contacts);
        if (this->_cpIterator.getNextChange(0) >= 0) {
            this->scheduleAt(this->_cpIterator.getNextChange(0),
                    this->_selfMsg);
        }
    } else if (stage == 1) {
        // do nothing wait for next stage
    } else if (stage == 2) {
        /*std::vector<NodeBase *> nodes = NodeRegistry::getInstance()->getNodes();
         for (NodeBase *node : nodes) {
         std::vector<contact_plan_entry_t *> allContacts =
         this->_cpIterator.getAllContacts();
         // we're adding the contacts in reverse, since ION doesn't add all ranges
         // otherwise, because it thinks they are overlapping, when they aren't
         ContactPlanEntryNodeFirstComparator cmp;
         std::stable_sort(allContacts.begin(), allContacts.end(), cmp);
         for (auto i = allContacts.rbegin(); i != allContacts.rend(); ++i) {
         contact_plan_entry_t *contact = *i;
         if (contact->sourceNodeId == contact->sinkNodeId) {
         continue;
         }
         node->addContactBetween(contact->startTime, contact->endTime,
         contact->sourceNodeId, contact->sinkNodeId,
         contact->bitrate, contact->range);
         }
         }*/
    }
}

void ContactPlanManager::handleMessage(omnetpp::cMessage *msg) {
    if (msg == this->_selfMsg) {
        EV_INFO << "ContactPlanManager::handleMessage" << omnetpp::endl;
        //printf("ContactPlanManager::handleMessage\n");
        NodeRegistry *nodeRegistry = NodeRegistry::getInstance();
        std::vector<NodeBase*> nodes = NodeRegistry::getInstance()->getNodes();
        int64_t currentSimTime = omnetpp::simTime().inUnit(omnetpp::SIMTIME_S);

        // notifying all nodes that limboed packages might become routable again
        if (this->par("shouldWaitForLimboFlush").boolValue()) {
            // notifying nodes that they should recheck packets in limbo
            for (NodeBase *node : nodes) {
                node->waitForLimboDecisions(true);
            }
        }

        // looking for contact plan changes
        EV_INFO
                       << "ContactPlanManager::handleMessage looking for contact plan changes"
                       << omnetpp::endl;
        std::vector<contact_plan_entry_t*> contactChanges =
                this->_cpIterator.getChangesUntil(currentSimTime);
        // notifying nodes of contact changes for tracking
        for (auto contact : contactChanges) {
            // check that we have at least the source node
            if (nodeRegistry->getNode(contact->sourceNodeId) == nullptr) {
                EV_WARN << "ContactPlanManager::handleMessage: no source node "
                               << contact->sourceNodeId << omnetpp::endl;
                continue;
            }
            // we're skipping loopback contacts
            if (contact->sourceNodeId == contact->sinkNodeId) {
                continue;
            }

            if (contact->endTime > currentSimTime) {
                // new contact
                nodeRegistry->getNode(contact->sourceNodeId)->addContactTo(
                        contact->sinkNodeId);
                if (nodeRegistry->isGroundStation(contact->sinkNodeId)) {
                    nodeRegistry->getNode(contact->sinkNodeId)->addContactTo(
                            contact->sourceNodeId);
                }
            } else {
                // expired contact
                nodeRegistry->getNode(contact->sourceNodeId)->removeContactTo(
                        contact->sinkNodeId);
                if (nodeRegistry->isGroundStation(contact->sinkNodeId)) {
                    nodeRegistry->getNode(contact->sinkNodeId)->removeContactTo(
                            contact->sourceNodeId);
                }
            }
        }

        // we're waiting until all nodes had a chance to check if any packets in
        // limbo are now deliverable
        if (this->par("shouldWaitForLimboFlush").boolValue()) {
            int numNodes = nodes.size();
            EV_INFO << "waiting for " << numNodes
                           << " nodes to flush their limbo" << omnetpp::endl;
            //printf("ContactPlanManager: waiting for %d nodes to flush their limbo\n",
            //       numNodes);
            shouldFlushLimbo(1);
            this->_semaphore.waitForZero(numNodes);
            shouldFlushLimbo(0);
            EV_INFO << "all " << numNodes << " nodes to flush their limbo"
                           << omnetpp::endl;
            //printf("ContactPlanManager: all %d nodes to flush their limbo\n",
            //       numNodes);

            EV_INFO << "waiting for " << numNodes
                           << " nodes to check limboed packets"
                           << omnetpp::endl;
            //printf(
            //    "ContactPlanManager: waiting for %d nodes to check limboed packets\n",
            //    numNodes);
            for (NodeBase *node : nodes) {
                node->waitForLimboDecisions();
            }
            EV_INFO << "all " << numNodes << " nodes checked limboed packets"
                           << omnetpp::endl;
            //printf("ContactPlanManager: all %d nodes checked limboed packets\n",
            //       numNodes);
        }

        // figure out when the next change is
        int64_t nextChange = this->_cpIterator.getNextChange(currentSimTime);
        if (nextChange > 0) {
            this->scheduleAt(nextChange, this->_selfMsg);
        }
    } else {
        throw omnetpp::cRuntimeError("Unknown message");
    }
}

// private helper function which doesn't need any class context
static const contact_plan_entry_t*
findContactInVectorFor(const std::vector<contact_plan_entry_t*> &contactVector,
        unsigned int sourceNodeId, unsigned int sinkNodeId) {
    const contact_plan_entry_t *rtn = nullptr;

    for (const contact_plan_entry_t *contact : contactVector) {
        if (contact->sourceNodeId == sourceNodeId
                && contact->sinkNodeId == sinkNodeId) {
            if (rtn == nullptr) {
                rtn = contact;
            } else if (contact->endTime < rtn->startTime) {
                rtn = contact;
            }
        }
    }
    return rtn;
}

static const contact_plan_entry_t*
findContactInVectorFor(const std::vector<contact_plan_entry_t*> &contactVector,
        unsigned int sourceNodeId) {
    const contact_plan_entry_t *rtn = nullptr;
    for (const contact_plan_entry_t *contact : contactVector) {
        if (contact->sourceNodeId == sourceNodeId
                && contact->sinkNodeId != sourceNodeId) {
            if (rtn == nullptr) {
                rtn = contact;
            } else if (contact->endTime < rtn->startTime) {
                rtn = contact;
            }
        }
    }
    return rtn;
}

const std::vector<contact_plan_entry_t*> ContactPlanManager::getAllContacts() const {
    return this->_cpIterator.getAllContacts();
}

const std::vector<contact_plan_entry_t*> ContactPlanManager::getActiveContacts() const {
    return this->_cpIterator.getActiveContacts();
}

const contact_plan_entry_t*
ContactPlanManager::getActiveContactFor(unsigned int sourceNodeId,
        unsigned int sinkNodeId) const {
    return findContactInVectorFor(this->_cpIterator.getActiveContacts(),
            sourceNodeId, sinkNodeId);
}

const contact_plan_entry_t*
ContactPlanManager::getActiveContactFor(unsigned int sourceNodeId) const {
    return findContactInVectorFor(this->_cpIterator.getActiveContacts(),
            sourceNodeId);
}

bool ContactPlanManager::hasActiveContactFor(unsigned int sourceNodeId,
        unsigned int sinkNodeId) const {
    return this->getActiveContactFor(sourceNodeId, sinkNodeId) != nullptr;
}

const contact_plan_entry_t*
ContactPlanManager::getUpcomingContactFor(unsigned int sourceNodeId,
        unsigned int sinkNodeId) const {
    return findContactInVectorFor(this->_cpIterator.getUpcomingContacts(),
            sourceNodeId, sinkNodeId);
}

const contact_plan_entry_t*
ContactPlanManager::getUpcomingContactFor(unsigned int sourceNodeId) const {
    return findContactInVectorFor(this->_cpIterator.getUpcomingContacts(),
            sourceNodeId);
}

bool ContactPlanManager::hasUpcomingContactFor(unsigned int sourceNodeId,
        unsigned int sinkNodeId) const {
    return this->getUpcomingContactFor(sourceNodeId, sinkNodeId) != nullptr;
}

const contact_plan_entry_t*
ContactPlanManager::getNextContactFor(unsigned int sourceNodeId,
        unsigned int sinkNodeId) const {
    const contact_plan_entry_t *rtn = this->getActiveContactFor(sourceNodeId,
            sinkNodeId);
    if (rtn == nullptr) {
        rtn = this->getUpcomingContactFor(sourceNodeId, sinkNodeId);
    }
    return rtn;
}

const contact_plan_entry_t*
ContactPlanManager::getNextContactFor(unsigned int sourceNodeId) const {
    const contact_plan_entry_t *rtn = this->getActiveContactFor(sourceNodeId);
    if (rtn == nullptr) {
        rtn = this->getUpcomingContactFor(sourceNodeId);
    }
    return rtn;
}

bool ContactPlanManager::hasNextContactFor(unsigned int sourceNodeId,
        unsigned int sinkNodeId) const {
    return this->getNextContactFor(sourceNodeId, sinkNodeId) != nullptr;
}

// WARNING: this method can't do any EV_ logging, as it is called asynchronously
// from different threads and causes double-free errors
void ContactPlanManager::limboFlushedNotification(unsigned int nodeId) {
    // Every node should have a chance to check its limbo, before a node does it
    // again, so no need to keep track of the nodeIds we already got.
    printf("ContactPlanManager: Got limboFlushedNotification from node %d\n",
            nodeId);
    this->_semaphore.notify();
}

void ContactPlanManager::finish() {
    omnetpp::cSimpleModule::finish();
}

}  // namespace estnet
