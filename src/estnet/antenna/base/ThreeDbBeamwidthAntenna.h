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

#ifndef __ANTENNAS_THREEDB_BEAMWIDTH_ANTENNA_H__
#define __ANTENNAS_THREEDB_BEAMWIDTH_ANTENNA_H__

#include <inet/common/Units.h>
#include <inet/common/Ptr.h>

#include "AntennaBaseWithOrientation.h"

namespace estnet {

/**
 * Base class for antennas modeled via
 * a 3dB Beamwidth.
 */
class ESTNET_API ThreeDbBeamwidthAntenna: public AntennaBaseWithOrientation {
protected:
    /** @brief initialization method */
    virtual void initialize(int stage) override;

    class ThreeDbAntennaGain: public inet::physicallayer::IAntennaGain {
    public:
        ThreeDbAntennaGain(double maxGain, double minGain,
                inet::units::values::deg beamWidth);
        virtual double getMinGain() const override {
            return minGain;
        }
        virtual double getMaxGain() const override {
            return maxGain;
        }
        virtual inet::units::values::deg getBeamWidth() const {
            return beamWidth;
        }
        virtual double computeGain(const inet::Quaternion direction) const
                override;

    protected:
        double maxGain;
        double minGain;
        inet::units::values::deg beamWidth;
    };

    inet::Ptr<ThreeDbAntennaGain> gain;

public:
    /** @brief Prints this object to the provided output stream */
    virtual std::ostream& printToStream(std::ostream &stream, int level) const
            override;
    /** @brief Returns the maximum possible gain of the antenna */
    virtual inet::Ptr<const inet::physicallayer::IAntennaGain> getGain() const
            override {
        return gain;
    }
    /** @brief Returns the Beamwidth of the Antenna*/
    virtual inet::units::values::deg getBeamWidth();
};

}  // namespace estnet

#endif
