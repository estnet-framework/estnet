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

#ifndef __UTILS__MODULE_ACCESS_TEST_H__
#define __UTILS__MODULE_ACCESS_TEST_H__

#include <omnetpp.h>
#include "inet/networklayer/common/InterfaceEntry.h"

/**
 * Copied functions from inet, since the dllimport would not work correctly.
 */

namespace estnet {

/**
 * Gets a module in the module tree, given by its absolute or relative path
 * defined by 'par' parameter.
 * Returns the pointer to a module of type T or throws an error if module not found
 * or type mismatch.
 */
template<typename T>
T *getModuleFromPar(omnetpp::cPar& par, const omnetpp::cModule *from, bool required = true);

template<typename T>
T *getModuleFromPar(omnetpp::cPar& par, const omnetpp::cModule *from, bool required)
{
    const char *path = par;
    omnetpp::cModule *mod = from->getModuleByPath(path);
    if (!mod) {
        if (required)
            throw omnetpp::cRuntimeError("Module not found on path '%s' defined by par '%s'", path, par.getFullPath().c_str());
        else
            return nullptr;
    }
    T *m = dynamic_cast<T *>(mod);
    if (!m)
        throw omnetpp::cRuntimeError("Module can not cast to '%s' on path '%s' defined by par '%s'", omnetpp::opp_typename(typeid(T)), path, par.getFullPath().c_str());
    return m;
}

/**
 * Returns true if the given module is a network node, i.e. a module
 * with the @networkNode property set.
 */
bool isNetworkNode(const omnetpp::cModule *mod);

/**
 * Find the node containing the given module.
 * Returns nullptr, if no containing node.
 */
omnetpp::cModule *findContainingNode(const omnetpp::cModule *from);

/**
 * Find the nic module (inside the networkNode) containing the given module.
 * Returns nullptr, if no containing nic module.
 */
inet::InterfaceEntry *findContainingNicModule(const omnetpp::cModule *from);

}  // namespace estnet

#endif
