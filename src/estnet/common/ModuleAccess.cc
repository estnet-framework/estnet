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

#include "ModuleAccess.h"

/**
 * Copied functions from inet, since the dllimport would not work correctly.
 */

namespace estnet {

class InterfaceEntry;

inline bool _isNetworkNode(const omnetpp::cModule *mod)
{
    omnetpp::cProperties *props = mod->getProperties();
    return props && props->getAsBool("networkNode");
}

bool isNetworkNode(const omnetpp::cModule *mod)
{
    return (mod != nullptr) ? _isNetworkNode(mod) : false;
}

omnetpp::cModule *findContainingNode(const omnetpp::cModule *from)
{
    for (omnetpp::cModule *curmod = const_cast<omnetpp::cModule *>(from); curmod; curmod = curmod->getParentModule()) {
        if (_isNetworkNode(curmod))
            return curmod;
    }
    return nullptr;
}

inet::InterfaceEntry *findContainingNicModule(const omnetpp::cModule *from)
{
    for (omnetpp::cModule *curmod = const_cast<omnetpp::cModule *>(from); curmod; curmod = curmod->getParentModule()) {
        if (auto interfaceEntry = dynamic_cast<inet::InterfaceEntry *>(curmod))
            return interfaceEntry;
        omnetpp::cProperties *props = curmod->getProperties();
        if (props && props->getAsBool("networkNode"))
            break;
    }
    return nullptr;
}

}
