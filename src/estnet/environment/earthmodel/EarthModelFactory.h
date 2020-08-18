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

#ifndef __EARTH_MODEL_EARTH_MODEL_FACTORY_H__
#define __EARTH_MODEL_EARTH_MODEL_FACTORY_H__

#include "EarthModelSphere.h"
#include "EarthModelWGS84.h"
#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

class ESTNET_API EarthModelFactory {
public:
    /**
     * Available earth models
     */
    enum EarthModels {
        SPHERE, WGS84
    };

    /**
     * @brief Returns a new instance of the given earth model
     */
    static IEarthModel* get(EarthModels model) {
        if (model == SPHERE) {
            return new EarthModelSphere();
        }
        if (model == WGS84) {
            return new EarthModelWGS84();
        }
        return nullptr;
    }
};

}  // namespace estnet

#endif
