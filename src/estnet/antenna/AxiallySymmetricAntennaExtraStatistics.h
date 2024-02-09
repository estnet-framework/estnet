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

#ifndef __AXIALLY_SYMMETRIC_ANTENNA_EXTRA_STATISTICS_H__
#define __AXIALLY_SYMMETRIC_ANTENNA_EXTRA_STATISTICS_H__

#include "AxiallySymmetricAntennaExtraStatistics.h"

#include <inet/physicallayer/antenna/AxiallySymmetricAntenna.h>

/*
 * Copy of inet::physicallayer::AxiallySymmetricAntenna with added
 * recording of the gain and fix of the gain calculation
 */
namespace estnet {

class AxiallySymmetricAntennaExtraStatistics: public inet::physicallayer::AxiallySymmetricAntenna {
private:
    static omnetpp::simsignal_t gainSiganl;
public:
    class AntennaGainExtraStatistics: public inet::physicallayer::IAntennaGain {
    protected:
        double minGain = NaN;
        double maxGain = NaN;
        inet::Coord axisOfSymmetryDirection = inet::Coord::NIL;
        std::map<inet::rad, double> gainMap;
        AxiallySymmetricAntennaExtraStatistics *parent;

    public:
        AntennaGainExtraStatistics(
                AxiallySymmetricAntennaExtraStatistics *parent,
                const char *axis, double baseGain, const char *gains);

        virtual double getMinGain() const override {
            return minGain;
        }
        virtual double getMaxGain() const override {
            return maxGain;
        }
        virtual double computeGain(const inet::Quaternion direction) const
                override;
    };

    inet::Ptr<AntennaGainExtraStatistics> gain;

protected:
    virtual void initialize(int stage) override;

public:
    virtual inet::Ptr<const inet::physicallayer::IAntennaGain> getGain() const
            override {
        return gain;
    }
    ;

};

}  // namespace estnet

#endif /* __AXIALLY_SYMMETRIC_ANTENNA_EXTRA_STATISTICS_H__ */
