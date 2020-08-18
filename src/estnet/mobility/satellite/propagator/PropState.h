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

#ifndef SATMOBILITY_PROPAGATORS_PROPSTATE_H_
#define SATMOBILITY_PROPAGATORS_PROPSTATE_H_

#include <omnetpp.h>
#include <memory> // std::shared_ptr
#include "estnet/common/time/GlobalJulianDate.h"

namespace estnet {

class ESTNET_API PropState;
typedef std::shared_ptr<PropState> tPropState_Ptr; // Reference counting pointer to a PropState

class ESTNET_API PropState {
public:
    PropState() {
    }

    /**
     * Casts the input argument cstate into the type of this state.
     * @param cstate pointer to the datavalue, that should be cast into this class
     * type
     * @return cstate, as castet into this state class type
     */
    template<class sType> static sType* as(PropState *cstate) {
        // check if cstate is of the correct state type and not a NULL pointer
        sType *tmpState = nullptr;
        if (!cstate) {
            throw omnetpp::cRuntimeError(
                    "Trying to convert a NULL pointer to a state of type '%s'!",
                    omnetpp::opp_typename(typeid(sType)));
        }
        if (!(tmpState = dynamic_cast<sType*>(cstate))) {
            throw omnetpp::cRuntimeError(
                    "Trying to convert a state of wrong type "
                            "'%s' to a state of type '%s'",
                    omnetpp::opp_typename(typeid(*cstate)),
                    omnetpp::opp_typename(typeid(sType)));
        }
        return tmpState;
    }

    /**
     * Casts the input argument cstate state shared pointer into the state shared
     * pointer type of another type.
     * @param cstate shared pointer of a PropState, that should be cast into a
     * shared pointer of another PropState type
     * @return a shared pointer as the new type, but pointing to the original
     * object. The use count is increased by one. If the conversion fails, a
     * shared pointer to a NULL object is returned.
     */
    template<class sType>
    static std::shared_ptr<sType> asPtr(tPropState_Ptr &cstate) {
        // check if cstate is of the correct state type and not a NULL pointer
        std::shared_ptr<sType> tmpState(nullptr);
        if (!cstate.get()) {
            throw omnetpp::cRuntimeError(
                    "Trying to convert a NULL shared pointer to "
                            "a shared pointer of type '%s'!",
                    omnetpp::opp_typename(typeid(sType)));
        }
        tmpState = std::dynamic_pointer_cast<sType>(cstate);
        if (!tmpState.get()) {
            throw omnetpp::cRuntimeError(
                    "Trying to convert a state of wrong type");
            // throw omnetpp::cRuntimeError("Trying to convert a state of wrong type
            // '%s' to a state of type
            // '%s'",opp_typename(typeid(*(cstate.get()))),opp_typename(typeid(sType)));
        }
        return tmpState;
    }

    /**
     * Returns the current time stamp of this state.
     * @return current time stamp of this state
     */
    virtual cJulian getTimeStamp() const {
        return _TimeStamp;
    }

protected:
    /**
     * Sets the time stamp of the state.
     * @param timeStamp new time stamp of the state
     */
    void setTimeStamp(cJulian const &timeStamp) {
        _TimeStamp = timeStamp;
    }
    cJulian _TimeStamp; // Current time stamp of the state

private:
};

}  // namespace estnet

#endif /* SATMOBILITY_PROPAGATORS_PROPSTATE_H_ */
