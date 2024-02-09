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

#include "MemorizedDataHandler.h"

#include <string>
#include <sstream>
#include <algorithm>
#include <cmath>

#include <inet/common/INETMath.h>
#include <inet/common/geometry/common/Coord.h>

#include "estnet/global_config.h"
#include "estnet/environment/contract/IEarthModel.h"

using std::cout;
using namespace std;

namespace estnet {

static MemorizedDataHandler *instance;

MemorizedDataHandler::~MemorizedDataHandler() {
    delete &file;
    delete &dataSet;
    delete &acquaintedShipMap;
    delete &messageToSendMap;
    delete satMobility;
    delete earthModel;
    delete instance;
}

MemorizedDataHandler* MemorizedDataHandler::getInstance(
        SatMobility *satMobility, IEarthModel *earthModel, std::string path) {
    if (instance == nullptr) {
        new MemorizedDataHandler(satMobility, earthModel, path);
    }
    return instance;
}

int64_t getCellNumber(double latitude, double longtitude, double resolution) {
    latitude = latitude - std::fmod(latitude, resolution);
    longtitude = longtitude - std::fmod(longtitude, resolution);
    return (((latitude + 90) / resolution) * 360 / resolution)
            + ((longtitude + 180) / resolution);
}

double sumDegLatitude(double summand1, double summand2) {
    double sum = summand1 + summand2;
    if (sum > 90) {
        sum = sum - 180;
    }

    return sum;
}

double sumDegLongtitude(double summand1, double summand2) {
    double retWert = summand1 + summand2;
    if (retWert > 180) {
        retWert = (retWert - 360);
    }

    return retWert;
}

double subDegLatitude(double minuend, double subtrahend) {
    double difference = minuend - subtrahend;
    if (difference < -90) {
        difference = difference + 180;
    }
    return difference;
}

double subDegLongtitude(double minuend, double subtrahend) {
    double difference = minuend - subtrahend;
    if (difference < -180) {
        difference = difference + 360;
    }
    return difference;
}

MemorizedDataHandler::MemorizedDataHandler(SatMobility *satMobility,
        IEarthModel *earthModel, std::string path) {
    if (instance == nullptr) {
        // reads data file
        file.open(path);
        if (file.is_open()) {
            std::string line;
            if (getline(file, line)) {
                std::size_t pos = line.find("=");
                std::string resString = line.substr(pos + 1);
                resolution = std::stod(resString);
            }

            // Iterates over all line in File
            double numTransmitter, totalNumOfTransmitters = 0;
            std::string longtitude, latitude, numTransmitterString;
            int64_t counter, cellNumber = 0;
            std::size_t longEndPoint, latEndPoint;
            while (getline(file, line)) {
                latEndPoint = line.find(";");
                longEndPoint = line.find(";", latEndPoint + 1);

                latitude = line.substr(0, latEndPoint);
                longtitude = line.substr(latEndPoint + 1,
                        longEndPoint - latEndPoint - 1);

                numTransmitterString = line.substr(longEndPoint + 1);
                numTransmitter = std::stod(numTransmitterString);

                counter++;
                cellNumber = getCellNumber(std::stod(latitude),
                        std::stod(longtitude), resolution);
                if ((int) dataSet.size() < cellNumber) {
                    dataSet.resize(cellNumber + 1);
                }
                dataSet[cellNumber] = numTransmitter;
                totalNumOfTransmitters += numTransmitter;
            }
            cout << "MemorizedDataHandler: Total read transmitters: "
                    << std::to_string(totalNumOfTransmitters) << std::endl;

            file.close();

            this->earthModel = earthModel;
            this->satMobility = satMobility;

            numberOfMsgToSend = 0;
        } else {
            std::ostringstream oss;
            oss << "Memorized data loader file: " << path << " not found.";
            const char *errorMsg = oss.str().c_str();
            throw omnetpp::cRuntimeError(errorMsg);
        }
    }
    instance = this;
}

void MemorizedDataHandler::getDataForPoint(double latitude, double longitude,
        double &multiplier) {
    if (numberOfMsgToSend == 0) {
        if (satMobility != nullptr) {
            messageToSendMap.clear();
            inet::Coord coord = this->satMobility->getCurrentPosition();
            inet::Coord vel = this->satMobility->getCurrentVelocity();
            setDataForPointInAcquaintedShip(coord, vel, latitude, longitude);

            numberOfMsgToSend = this->messageToSendMap.size();
        }
    } else {
        multiplier = this->messageToSendMap.size()
                / calcIdleInterval(latitude, longitude, 0, 0);
        numberOfMsgToSend--;
    }
}

void MemorizedDataHandler::getDataForCone(double latitude, double longitude,
        double altitude, double beamWidth, double &multiplier) {
    if (numberOfMsgToSend == 0) {
        if (satMobility != nullptr) {
            messageToSendMap.clear();
            inet::Coord coord = this->satMobility->getCurrentPosition();
            inet::Coord vel = this->satMobility->getCurrentVelocity();
            setDataForConeInAcquaintedShip(coord, vel, latitude, longitude,
                    altitude, beamWidth);

            numberOfMsgToSend = this->messageToSendMap.size();
        }
    } else {
        multiplier = this->messageToSendMap.size()
                / calcIdleInterval(latitude, longitude, altitude, beamWidth);
        numberOfMsgToSend--;
    }
}

double MemorizedDataHandler::calcIdleInterval(double latitude, double longitude,
        double altitude, double beamWidth) {
    if (satMobility == nullptr) {
        return 0.0;
    }
    return ((EARTH_AVG_R * inet::math::deg2rad((90 - beamWidth) * 2)))
            / satMobility->getCurrentVelocity().length();
}

void calcSightDuration(inet::Coord coordShip, inet::Coord speedCoord,
        inet::Coord coorNadirSat, IEarthModel *earthModel,
        double ConeFootPrintRadius, double *sightDuration) {
    // Vector from ship to satellite nadir
    inet::Coord vectorSatShip = coordShip - coorNadirSat;

    // Angle between velVector and SatShip vector
    double cosAlpha = (speedCoord * vectorSatShip)
            / (sqrt(speedCoord.squareLength())
                    * sqrt(vectorSatShip.squareLength()));

    // Conversion in angle < 90°
    double angle;
    if (std::acos(cosAlpha) > PI + PI / 2) {
        angle = 2 * PI - std::acos(cosAlpha);
    } else if (std::acos(cosAlpha) > PI) {
        angle = std::acos(cosAlpha) - PI;
    } else if (std::acos(cosAlpha) > PI / 2) {
        angle = PI - std::acos(cosAlpha);
    } else {
        angle = std::acos(cosAlpha);
    }

    // sin part of the SatShip distance is the y-value of a quarter circle function.
    // The corresponding x would be half of this distance.
    double yValue = sin(angle) * coorNadirSat.distance(coordShip);
    double distance = 2
            * sqrt(std::pow(ConeFootPrintRadius, 2) - std::pow(yValue, 2));

    // Conversion into real distance considering the earths shape.
    double gamma = std::asin(distance / (2 * EARTH_AVG_R)) * 2;
    double realDist = 2 * PI * EARTH_AVG_R * (gamma / (2 * PI));
    *sightDuration = realDist / sqrt(speedCoord.squareLength());
}

bool isShipDetected() {
    return true;
}

double getRadiusInLatitutdeRad(double halfBeamWidth) {
    double halfBeamWidthInRad = inet::math::deg2rad(halfBeamWidth);
    // Beam width of the vertical footprint. Distance between single latitudes is equidistant.
    return (PI / 2 - halfBeamWidthInRad);
}

double getFootPrintRadiusInKm(double halfBeamWidth) {
    // Beam width of the vertical footprint. Distance between single latitudes is equidistant.
    double latitudeRadiusInRad = getRadiusInLatitutdeRad(halfBeamWidth);

    // Erth diameter without WGS84
    double earthDiameter = 2 * PI * EARTH_AVG_R;

    // Footprint radius in km calculation based on latitude,
    // because the distance between single latitudes is equidistant.
    return (earthDiameter / (2 * PI)) * latitudeRadiusInRad;
}

double getRadiusInLongtitudeRad(double halfBeamWidth, double latitude) {
    double footPrintRadiusInKm = getFootPrintRadiusInKm(halfBeamWidth);

    // Radius of the latitude circle
    double latitudeCircleRadius = abs(
            cos(inet::math::deg2rad(latitude)) * EARTH_AVG_R);

    // Diameter of circle
    double diameter = 2 * PI * latitudeCircleRadius;
    if (diameter < footPrintRadiusInKm) {
        return 2 * PI;
    } else {
        return footPrintRadiusInKm / (diameter / (2 * PI));
    }
}

double calculateNumOfMessagesToSend(double numTransmitterOfCurrentCell,
        double *numMessagesRemainder) {
    // Decimals are added together until they are 1
    *numMessagesRemainder += std::fmod(numTransmitterOfCurrentCell, 1.0);

    if (*numMessagesRemainder > 1) {
        *numMessagesRemainder = *numMessagesRemainder - 1;
        return numTransmitterOfCurrentCell
                - std::fmod(numTransmitterOfCurrentCell, 1.0) + 1;
    }

    return numTransmitterOfCurrentCell
            - std::fmod(numTransmitterOfCurrentCell, 1.0);
}

double getNumber(double minValue, double maxValue, double resolution) {
    // Number of rows/columns between the values corresponds to the resolution
    if (minValue > maxValue) {
        return ((180 - std::abs(maxValue) + 180 - std::abs(minValue))
                - std::fmod(
                        (180 - std::abs(maxValue) + 180 - std::abs(minValue)),
                        resolution)) / resolution;
    } else {
        return ((maxValue - minValue)
                - std::fmod((maxValue - minValue), resolution)) / resolution;
    }
}

void MemorizedDataHandler::setDataForConeInAcquaintedShip(inet::Coord coord,
        inet::Coord speedCoord, double latitude, double longitude,
        double altitude, double halfBeamWidth) {

    double latitudeRadiusInRad = inet::math::rad2deg(
            getRadiusInLatitutdeRad(halfBeamWidth));

    // Radius of the footprint in longitude radians along the circle based on the radius in km
    double longtitudeRadiusInRad = inet::math::rad2deg(
            getRadiusInLongtitudeRad(halfBeamWidth, latitude));

    // Iterates over a 2r by 2r square and finds all points inside a circle
    // with radius r and center point lat/lon
    double minLat = subDegLatitude(latitude, latitudeRadiusInRad);
    double maxLat = sumDegLatitude(latitude, latitudeRadiusInRad);

    double minLong = subDegLongtitude(longitude, longtitudeRadiusInRad);
    double maxLong = sumDegLongtitude(longitude, longtitudeRadiusInRad);

    std::string keyForMap;
    inet::Coord coordShip, vectorSatShip, coordSat, coorNadirSat;
    double sightDuration = 0;

    earthModel->convertLatLongHeightToECEF(
            inetu::deg(inet::math::deg2rad(latitude)),
            inetu::deg(inet::math::deg2rad(longitude)), inetu::m(10),
            coorNadirSat);

    double numColumns = getNumber(minLong, maxLong, resolution);
    double numRows = getNumber(minLat, maxLat, resolution);

    double cellNumber, value, numMessagesRemainder, workLat, workLong = 0;

    int total = 0;

    double roundTime = (2 * PI * (altitude + EARTH_AVG_R))
            / speedCoord.length();

    for (int i = 0; i <= numRows; i++) {
        for (int j = 0; j <= numColumns; j++) {
            workLat = sumDegLatitude(minLat, j * resolution);
            workLong = sumDegLongtitude(minLong, i * resolution);
            cellNumber = getCellNumber(workLat, workLong, resolution);
            earthModel->convertLatLongHeightToECEF(
                    inetu::deg(inet::math::deg2rad(workLat)),
                    inetu::deg(inet::math::deg2rad(workLong)), inetu::m(10),
                    coordShip);

            // Continue with the next dataset, if no data exists in the current cell
            // or the data points are more then radiusFootprint away from NadirPoint
            if (cellNumber >= dataSet.size()
                    || coorNadirSat.distance(coordShip)
                            > getFootPrintRadiusInKm(halfBeamWidth)) {
                continue;
            }

            value = calculateNumOfMessagesToSend(dataSet.at(cellNumber),
                    &numMessagesRemainder);

            for (int count = 0; count < value; count++) {
                keyForMap = "";
                keyForMap.append(std::to_string(cellNumber));
                keyForMap.append("_");
                keyForMap.append(std::to_string(count));

                // Value SIMTIME_ZERO means we have to send a message for this ship.
                // Logic: Add message to map if: it is unknown or the current time is greater then last hit time + duration of sight

                if (acquaintedShipMap.find(keyForMap)
                        == acquaintedShipMap.end()) {
                    acquaintedShipMap.insert(
                            std::pair<std::string, double>(keyForMap,
                                    omnetpp::simTime().dbl()));
                }

                if (acquaintedShipMap.find(keyForMap)->second
                        <= omnetpp::simTime().dbl()) {
                    totalDetected++;
                    total++;
                    acquaintedShipMap.find(keyForMap)->second =
                            omnetpp::simTime().dbl() + sightDuration
                                    + roundTime;

                    if (isShipDetected()) {
                        messageToSendMap.insert(
                                std::pair<std::string, omnetpp::simtime_t>(
                                        keyForMap, SIMTIME_ZERO));
                    }
                }
            }
        }
    }
}

void MemorizedDataHandler::setDataForPointInAcquaintedShip(inet::Coord coord,
        inet::Coord speedCoord, double latitude, double longitude) {
    inet::Coord coordShip, vectorSatShip, coordSat, coorNadirSat;

    earthModel->convertLatLongHeightToECEF(
            inetu::deg(inet::math::deg2rad(latitude)),
            inetu::deg(inet::math::deg2rad(longitude)), inetu::m(10),
            coorNadirSat);
    earthModel->convertLatLongHeightToECEF(
            inetu::deg(inet::math::deg2rad(latitude)),
            inetu::deg(inet::math::deg2rad(longitude)), inetu::m(10),
            coordShip);
    double cellNumber, value, numMessagesRemainder = 0;
    cellNumber = getCellNumber(latitude, longitude, resolution);

    // Continue with the next dataset, if no data exists in the current cell
    // or the data points are more then radiusFootprint away from NadirPoint
    if (cellNumber >= dataSet.size()
            || coorNadirSat.distance(coordShip) > resolution) {
        return;
    }
    value = calculateNumOfMessagesToSend(dataSet.at(cellNumber),
            &numMessagesRemainder);

    std::string keyForMap;
    int total = 0;

    for (int count = 0; count < value; count++) {
        keyForMap = "";
        keyForMap.append(std::to_string(cellNumber));
        keyForMap.append("_");
        keyForMap.append(std::to_string(count));

        // Value SIMTIME_ZERO means we have to send a message for this ship.
        // Logic: Add message to map if: it is unknown or the current time is greater then last hit time + duration of sight

        if (acquaintedShipMap.find(keyForMap) == acquaintedShipMap.end()) {
            acquaintedShipMap.insert(
                    std::pair<std::string, double>(keyForMap,
                            omnetpp::simTime().dbl()));
        }

        if (acquaintedShipMap.find(keyForMap)->second
                <= omnetpp::simTime().dbl()) {
            totalDetected++;
            total++;
            acquaintedShipMap.find(keyForMap)->second = omnetpp::simTime().dbl();

            if (isShipDetected()) {
                messageToSendMap.insert(
                        std::pair<std::string, omnetpp::simtime_t>(keyForMap,
                                SIMTIME_ZERO));
            }
        }
    }
}

}  // namespace estnet
