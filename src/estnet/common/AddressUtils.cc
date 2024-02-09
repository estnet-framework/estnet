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

#include "AddressUtils.h"

#include <inet/networklayer/common/InterfaceTable.h>

#include "estnet/common/node/NodeRegistry.h"

namespace estnet {

inet::MacAddress getMacAddressOfNode(unsigned int nodeNo,
        unsigned int radioNo) {
    // lowest common denominator is ipv4, so we support 3 byte node, 1-byte radio
    // number
    if (nodeNo > 16777215) {
        throw omnetpp::cRuntimeError("Too many nodes");
    }
    if (radioNo > 255) {
        throw omnetpp::cRuntimeError("Too many radios");
    }
    uint64_t macInt = 0x000000000000;
    macInt |= radioNo;
    macInt |= nodeNo << 8;
    return inet::MacAddress(macInt);
}

inet::Ipv4Address getIpAddressOfNode(unsigned int nodeNo,
        unsigned int radioNo) {
    NodeBase *node = NodeRegistry::getInstance()->getNode(nodeNo);
    inet::InterfaceTable *interfaceTable =
            dynamic_cast<inet::InterfaceTable *>(node->getSubmodule(
                    "networkHost")->getSubmodule("interfaceTable"));
    return interfaceTable->getInterface(0)->getNetworkAddress().toIpv4();
}

// TODO use interface table instead
static omnetpp::cNEDValue node_mac_address(omnetpp::cComponent *context,
        omnetpp::cNEDValue argv[], int argc) {
    long radio_no = argv[1];
    long node_no = argv[0];
    return getMacAddressOfNode(node_no, radio_no).str();
}
Define_NED_Function(node_mac_address,
        "int node_mac_address(int node_no, int radio_no)");

}  // namespace estnet
