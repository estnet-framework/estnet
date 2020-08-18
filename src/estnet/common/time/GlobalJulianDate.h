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

#ifndef __ESTNET_GLOBALJULIANDATE_H_
#define __ESTNET_GLOBALJULIANDATE_H_

#include "estnet/common/ESTNETDefs.h"
#include "cJulian.h"

using namespace omnetpp;

namespace estnet {

/**
 * Provides access to a global simulation time
 * Maps the simulation time to a julian date referred to a given start time
 */
class ESTNET_API GlobalJulianDate: public cSimpleModule {
public:
    GlobalJulianDate();
    ~GlobalJulianDate();
    static const GlobalJulianDate& getInstance();
    static const GlobalJulianDate* getInstancePtr();

    cJulian currentSimTime() const;
    cJulian simTime2JulianDate(double simTime) const;
    double julianDate2SimTime(const cJulian& simTime) const;
protected:
    virtual void initialize();
private:
    cJulian _simStart;
};

} //namespace

#endif
