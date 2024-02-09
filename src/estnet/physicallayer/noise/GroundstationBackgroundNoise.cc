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

#include <fstream>
#include <queue>
#include <algorithm>

#include "GroundstationBackgroundNoise.h"

#include "inet/physicallayer/analogmodel/packetlevel/ScalarNoise.h"
#include "inet/physicallayer/backgroundnoise/IsotropicScalarBackgroundNoise.h"
#include "inet/physicallayer/common/packetlevel/BandListening.h"

namespace estnet {

Define_Module(GroundstationBackgroundNoise);

void GroundstationBackgroundNoise::initialize(int stage) {
    cModule::initialize(stage);
    if (stage == 0) {
        minElevation = par("minElevation").doubleValue();
        interpolationMethod = par("interpolationMethod").intValue();
        if (interpolationMethod < 0 || interpolationMethod > 2)
            throw cRuntimeError(
                    "GroundstationBackgroundNoise: interpolation method %d is not valid.",
                    interpolationMethod);
        regularGrid = par("regularGrid").boolValue();
        if (interpolationMethod == 0 && regularGrid == false)
            throw cRuntimeError(
                    "GroundstationBackgroundNoise: selected interpolation method requires a regular grid.");
        numOfInterpolationNeighbors =
                par("numOfInterpolationNeighbors").intValue();

        // read in noise data
        std::ifstream file(par("noiseMapPath").stdstringValue());
        if (!file)
            throw cRuntimeError(
                    "GroundstationBackgroundNoise: map file not found.");
        std::string line;
        std::getline(file, line); // skip first line with text
        while (std::getline(file, line)) { // read file line by line
            std::stringstream ss(line);
            std::string s;
            std::getline(ss, s, ';'); // get separate values
            noiseElement e;
            e.e_min = std::stod(s) * RADS_PER_DEG;
            std::getline(ss, s, ';');
            e.e_max = std::stod(s) * RADS_PER_DEG;
            std::getline(ss, s, ';');
            e.a_min = std::stod(s) * RADS_PER_DEG;
            std::getline(ss, s, ';');
            e.a_max = std::stod(s) * RADS_PER_DEG;
            std::getline(ss, s, ';');
            e.power = pow(10.0, (std::stod(s) - 30.0) / 10.0); // convert from dBm to W

            if (interpolationMethod != 1) { // bilinear Interpolation or NDDNISD interpolation
                data.push_back(e);
            } else { // thin plate splines interpolation
                MultivariateSplines::DenseVector x(3);
                double r = 1;
                // convert to Cartesian coordinates
                x(0) = r * sin((e.e_max + e.e_min) * 0.5)
                        * cos((e.a_max + e.a_min) * 0.5);
                x(1) = r * sin((e.e_max + e.e_min) * 0.5)
                        * sin((e.a_max + e.a_min) * 0.5);
                x(2) = r * cos((e.e_max + e.e_min) * 0.5);
                samples.addSample(x, e.power);
            }
        }

        file.close();
        if (interpolationMethod == 1)
            rbfspline =
                    new MultivariateSplines::RBFSpline(samples,
                            MultivariateSplines::RadialBasisFunctionType::THIN_PLATE_SPLINE);
        else if (interpolationMethod == 0) {
            struct point {
                double e, a;
                double power; // noise power [W]
            };

            halfAzimuthRes = 0.5 * (data[0].a_max - data[0].a_min);
            // smallest elevation in the new grid
            double minE = (data[0].e_max + data[0].e_min) * 0.5;
            // largest elevation in the new grid
            double maxE = (data[data.size() - 1].e_max
                    + data[data.size() - 1].e_min) * 0.5;

            // figure out the number of resolution steps in azimuth
            bool done = false;
            int n_az = 0;
            double firstElement = data[0].a_min;

            // convert the data from a cell with a data-point for the entire cell
            // to a single point in the center of the cell
            std::vector<point> centeredData;
            for (int i = 0; i < data.size(); ++i) {
                point p;
                // average the old bounds to get the center point of the grid cell
                p.a = (data[i].a_max + data[i].a_min) * 0.5;
                p.e = (data[i].e_max + data[i].e_min) * 0.5;

                // account for the periodicity of the azimuth by shifting the first element to the back
                if (p.a <= halfAzimuthRes)
                    p.a += 2.0 * M_PI;

                // expand the cell with the lowest elevation to the bottom
                if (p.e <= minE)
                    p.e = minElevation;
                // expand the cell with the largest elevation to 90 deg
                if (p.e >= maxE)
                    p.e = M_PI_2;

                p.power = data[i].power;
                centeredData.push_back(p);

                if (!done) {
                    if (n_az != 0) {
                        if (data[i].a_min == firstElement) {
                            done = true;
                            continue;
                        }
                    }
                    ++n_az;
                }
            }

            // convert the points to a grid, where every corner is a data-point
            for (int i = 0; i < centeredData.size() - n_az; ++i) {
                noiseGridElement e;
                e.power0 = centeredData[i].power;
                int shift = 0;
                if ((i + 1) % n_az == 0)
                    shift = -n_az;
                e.power1 = centeredData[i + 1 + shift].power;
                e.power2 = centeredData[i + n_az].power;
                e.power3 = centeredData[i + n_az + 1 + shift].power;

                e.e_min = centeredData[i].e;
                e.a_min = centeredData[i].a;
                if (e.a_min > 2.0 * M_PI)
                    e.a_min -= 2.0 * M_PI;
                e.e_max = centeredData[i + n_az].e;
                e.a_max = centeredData[i + 1].a;

                grid.push_back(e);
            }
        }

    }
}

double GroundstationBackgroundNoise::computeAngularDistance(double e0,
        double a0, double e1, double a1) const {
    // use spherical law of cosines
    return acos(sin(e0) * sin(e1) + cos(e0) * cos(e1) * cos(a1 - a0));
}

std::pair<double, double> GroundstationBackgroundNoise::avarageLatLong(
        std::vector<std::pair<double, double> > &latLong) const {
    double x = 0, y = 0, z = 0;
    for (auto l : latLong) { // go through all values
        // convert to Cartesian coordinates
        x += sin(l.first) * cos(l.second);
        y += sin(l.first) * sin(l.second);
        z += cos(l.first);
    }
    return std::make_pair(atan2(z, sqrt(x * x + y * y)), atan2(-y, x)); // convert back to latitude and longitude
}

const inet::physicallayer::INoise* GroundstationBackgroundNoise::computeNoise(
        const inet::physicallayer::IListening *listening) const {
    const inet::physicallayer::IAntenna *receiverAntenna =
            listening->getReceiver()->getAntenna();
    StaticTerrestrialMobility *mobility =
            dynamic_cast<StaticTerrestrialMobility*>(dynamic_cast<const omnetpp::cModule*>(receiverAntenna)->getParentModule() // radio
            ->getParentModule() // wlan
            ->getParentModule() // networkHost
            ->getSubmodule("mobility"));

    // get the current azimuth and elevation of the ground station
    double azimuth, elevation;
    mobility->getCurrentOrientationAngles(azimuth, elevation);

    double power = INFINITY;
    // if the elevation is below minElevation return infinity
    if (elevation >= minElevation) {
        // normalize between 0 and 2 pi
        if (azimuth < 0)
            azimuth += TWOPI;

        // choose between different interpolation methods
        switch (interpolationMethod) {
        case 0: { // bilinear Interpolation
            double x1 = 0, y1 = 0, x2 = 0, y2 = 0, Q11, Q12, Q21, Q22;

            // account for the periodicity of the azimuth by shifting the first element to the back
            if (azimuth <= halfAzimuthRes)
                azimuth += 2.0 * M_PI;

            // find the grid cell, where the current orientation is in
            for (int i = 0; i < grid.size(); ++i) {
                if (grid[i].e_min < elevation && grid[i].e_max > elevation
                        && grid[i].a_min < azimuth && grid[i].a_max > azimuth) {
                    x1 = grid[i].a_min;
                    y1 = grid[i].e_min;
                    x2 = grid[i].a_max;
                    y2 = grid[i].e_max;

                    Q11 = grid[i].power0;
                    Q21 = grid[i].power1;
                    Q12 = grid[i].power2;
                    Q22 = grid[i].power3;
                    break;
                }
            }

            // perform the bilinear interpolation
            double x = azimuth;
            double y = elevation;

            double f_x_y1 = (x2 - x) / (x2 - x1) * Q11
                    + (x - x1) / (x2 - x1) * Q21;
            double f_x_y2 = (x2 - x) / (x2 - x1) * Q12
                    + (x - x1) / (x2 - x1) * Q22;

            power = (y2 - y) / (y2 - y1) * f_x_y1
                    + (y - y1) / (y2 - y1) * f_x_y2;
            break;
        }
        case 1: { // thin plate splines interpolation
            MultivariateSplines::DenseVector x(3);
            double r = 1;
            x(0) = r * sin(elevation) * cos(azimuth);
            x(1) = r * sin(elevation) * sin(azimuth);
            x(2) = r * cos(elevation);

            power = rbfspline->eval(x);
            std::cout << power << endl;
            break;
        }
        case 2: { // NDDNISD interpolation
            // create a sorted queue with the distance as first element and the id as second element
            std::priority_queue<std::pair<double, int>,
                    std::vector<std::pair<double, int> >,
                    std::greater<std::pair<double, int> > > closestPoints;

            // go through every data point and compute the distance
            for (int i = 0; i < data.size(); ++i) {
                closestPoints.push(
                        std::make_pair(
                                computeAngularDistance(elevation, azimuth,
                                        (data[i].e_max + data[i].e_min) * 0.5, // use center of square as data point
                                        (data[i].a_max + data[i].a_min) * 0.5),
                                i));
            }
            int n = numOfInterpolationNeighbors; // num of points to consider

            // extract the closest neighbors from the priority queue
            std::pair<double, int> bestPoints[n];
            for (int i = 0; i < n; ++i) {
                bestPoints[i] = closestPoints.top();
                closestPoints.pop();
            }

            // compute normalized inverse squared distance
            double inverseDistances[n];
            double sum = 0;
            for (int i = 0; i < n; ++i) {
                inverseDistances[i] = 1.0
                        / (bestPoints[i].first * bestPoints[i].first);
                sum += inverseDistances[i];
            }
            for (int i = 0; i < n; ++i) {
                inverseDistances[i] /= sum;
            }

            // average the lat and long of the neighbors
            std::vector<std::pair<double, double> > latLong;
            for (int i = 0; i < n; ++i) {
                latLong.push_back(
                        std::make_pair(
                                (data[bestPoints[i].second].e_min
                                        + data[bestPoints[i].second].e_max)
                                        * 0.5,
                                (data[bestPoints[i].second].a_min
                                        + data[bestPoints[i].second].a_max)
                                        * 0.5));
            }
            std::pair<double, double> centroid = avarageLatLong(latLong);

            // compute the distances from the centroid to the neighbors
            double eta[n];
            for (int i = 0; i < n; ++i) {
                eta[i] = computeAngularDistance(centroid.first, centroid.second,
                        (data[bestPoints[i].second].e_min
                                + data[bestPoints[i].second].e_max) * 0.5,
                        (data[bestPoints[i].second].a_min
                                + data[bestPoints[i].second].a_max) * 0.5);
            }

            double weights[n];
            sum = 0;
            for (int i = 0; i < n; ++i) {
                weights[i] = eta[i] * inverseDistances[i];
                sum += weights[i];
            }
            for (int i = 0; i < n; ++i) {
                weights[i] /= sum;
            }

            power = 0;
            for (int i = 0; i < n; ++i) {
                power += weights[i] * data[bestPoints[i].second].power;
            }
            break;
        }
        default:
            throw cRuntimeError(
                    "GroundstationBackgroundNoise: interpolation method %d is not valid.",
                    interpolationMethod);
            break;
        }

    }

    // convert the noise power to the right return type
    const inet::physicallayer::BandListening *bandListening =
            omnetpp::check_and_cast<const inet::physicallayer::BandListening*>(
                    listening);
    omnetpp::simtime_t startTime = listening->getStartTime();
    omnetpp::simtime_t endTime = listening->getEndTime();
    std::map<omnetpp::simtime_t, inet::units::values::W> *powerChanges =
            new std::map<omnetpp::simtime_t, inet::units::values::W>();
    powerChanges->insert(
            std::pair<omnetpp::simtime_t, inet::units::values::W>(startTime,
                    power));
    powerChanges->insert(
            std::pair<omnetpp::simtime_t, inet::units::values::W>(endTime,
                    -power));

    return new inet::physicallayer::ScalarNoise(startTime, endTime,
            bandListening->getCenterFrequency(), bandListening->getBandwidth(),
            powerChanges);
}

} //estnet
