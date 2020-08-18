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

#ifndef __APPS__MEMORIZED_DATA_HANDLER_H__
#define __APPS__MEMORIZED_DATA_HANDLER_H__

#include <fstream>

#include <inet/common/geometry/common/Coord.h>

#include "estnet/application/contract/IPositionData.h"
#include "estnet/mobility/satellite/SatMobility.h"
#include "estnet/environment/contract/IEarthModel.h"

namespace estnet {

class ESTNET_API MemorizedDataHandler: public IPositionData {
public:
    MemorizedDataHandler(SatMobility *satMobility, IEarthModel *earthModel,
            std::string path);
    virtual ~MemorizedDataHandler();
    /** @brief gives multiplier for one specific longitude and latitude*/
    virtual void getDataForPoint(double latitude, double longitude,
            double &multiplier) override;
    /** @ brief gives the sum of all multipliers inside the beam*/
    virtual void getDataForCone(double latitude, double longitude,
            double altitude, double beamWidth, double &multiplier) override;

    virtual void setDataForConeInAcquaintedShip(inet::Coord coord,
            inet::Coord speedCoord, double latitude, double longitude,
            double altitude, double beamWidth);
    virtual double calcIdleInterval(double latitude, double longitude,
            double altitude, double beamWidth);
    static MemorizedDataHandler* getInstance(SatMobility *satMobility,
            IEarthModel *earthModel, std::string path);

private:
    std::vector<double> dataSet;
    std::ifstream file;
    double resolution;
    int totalDetected = 0;
    // List of known ships per satellite. KEY = 'Lat_Lon_Number'. VALUE = transmission time + duration of this flyover
    // Logic: Transmit only once data for each ship during each orbit
    std::map<std::string, double> acquaintedShipMap;
    // List of the messages to send during current orbit. KEY = 'Lat_Lon_Number'. VALUE = duration of this flyover
    std::map<std::string, omnetpp::simtime_t> messageToSendMap;
    SatMobility *satMobility = nullptr; // SatMobility module as position source
    IEarthModel *earthModel;
    int numberOfMsgToSend;
};

}  // namespace estnet

#endif
