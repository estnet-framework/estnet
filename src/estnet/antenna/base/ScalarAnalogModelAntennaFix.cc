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

#include "ScalarAnalogModelAntennaFix.h"

#include <inet/physicallayer/analogmodel/packetlevel/ScalarReception.h>
#include <inet/physicallayer/contract/packetlevel/IRadioMedium.h>

#include "estnet/mobility/satellite/common/EulerAngleHelpers.h"
#include "estnet/common/matrix/Matrix.h"

namespace estnet {

Define_Module(ScalarAnalogModelAntennaFix);

inet::Quaternion ScalarAnalogModelAntennaFix::computeTransmissionDirection(
        const inet::physicallayer::ITransmission *transmission,
        const inet::physicallayer::IArrival *arrival) const {
    const inet::Coord transmissionStartPosition =
            transmission->getStartPosition();
    const inet::Quaternion transmissionStartOrientation =
            transmission->getStartOrientation();
    const inet::Coord arrivalStartPosition = arrival->getStartPosition();

    inet::Quaternion transOrientationConj = transmissionStartOrientation;
    transOrientationConj.conjugate();

    inet::Coord transmissionVector = arrivalStartPosition
            - transmissionStartPosition;
    double distance = transmissionVector.length(); //for debug purposes
    transmissionVector.normalize();
    // Rotating the transmissionVector to our base frame makes the calculation easier
    transmissionVector = transOrientationConj.rotate(transmissionVector);

    // Antenna is pointed in X_Axis direction for RPY=(0,0,0)
    inet::Quaternion transmissionQuat = inet::Quaternion::rotationFromTo(
            inet::Coord::X_AXIS, transmissionVector);
    transmissionQuat.normalize();

    EV_TRACE << "ScalarAnalogModelAntennaFix::computeTransmissionDirection"
                    << " distance: " << distance
                    << " transmissionStartPosition "
                    << transmissionStartPosition
                    << " transmissionStartOrientation "
                    << transmissionStartOrientation
                    << " transmissionStartOrientationEuler "
                    << transmissionStartOrientation.toEulerAngles(true)
                    << " arrivalStartPosition " << arrivalStartPosition
                    << omnetpp::endl;

    return transmissionQuat;
}

inet::Quaternion ScalarAnalogModelAntennaFix::computeReceptionDirection(
        const inet::physicallayer::ITransmission *transmission,
        const inet::physicallayer::IArrival *arrival) const {
    const inet::Coord transmissionStartPosition =
            transmission->getStartPosition();
    const inet::Quaternion arrivalStartOrientation =
            arrival->getStartOrientation();
    const inet::Coord arrivalStartPosition = arrival->getStartPosition();

    inet::Quaternion arrOrientationConj = arrivalStartOrientation;
    arrOrientationConj.conjugate();

    inet::Coord arrivalVector = transmissionStartPosition
            - arrivalStartPosition;
    arrivalVector.normalize();
    arrivalVector = arrOrientationConj.rotate(arrivalVector);

    // Assumption: Antenna is pointing in X_AXIS for RPY=(0,0,0)
    inet::Quaternion arrivalQuat = inet::Quaternion::rotationFromTo(
            inet::Coord::X_AXIS, arrivalVector);
    arrivalQuat.normalize();

    EV_TRACE << "ScalarAnalogModelAntennaFix::computeReceptionDirectionQuat"
                    << " arrivalStartPosition " << arrivalStartPosition
                    << " arrivalStartOrientation " << arrivalStartOrientation
                    << " arrivalStartOrientationEuler"
                    << arrivalStartOrientation.toEulerAngles(true)
                    << " transmissionStartPosition "
                    << transmissionStartPosition << omnetpp::endl;

    return arrivalQuat;
}

inet::units::values::W ScalarAnalogModelAntennaFix::computeReceptionPower(
        const inet::physicallayer::IRadio *receiverRadio,
        const inet::physicallayer::ITransmission *transmission,
        const inet::physicallayer::IArrival *arrival) const {
    const inet::physicallayer::IRadioMedium *radioMedium =
            receiverRadio->getMedium();
    const inet::physicallayer::IRadio *transmitterRadio =
            transmission->getTransmitter();
    const inet::physicallayer::IAntenna *receiverAntenna =
            receiverRadio->getAntenna();
    const inet::physicallayer::IAntenna *transmitterAntenna =
            transmitterRadio->getAntenna();
    const inet::physicallayer::INarrowbandSignal *narrowbandSignalAnalogModel =
            omnetpp::check_and_cast<
                    const inet::physicallayer::INarrowbandSignal*>(
                    transmission->getAnalogModel());
    const inet::physicallayer::IScalarSignal *scalarSignalAnalogModel =
            omnetpp::check_and_cast<const inet::physicallayer::IScalarSignal*>(
                    transmission->getAnalogModel());

    // using Quaternions
    const inet::Coord receptionStartPosition = arrival->getStartPosition();
    const inet::Quaternion transmissionAntennaDirection =
            computeTransmissionDirection(transmission, arrival);
    const inet::Quaternion receptionAntennaDirection =
            computeReceptionDirection(transmission, arrival);
    EV_INFO << "ScalarAnalogModelAntennaFix transmissionAntennaDirection "
                   << transmissionAntennaDirection << omnetpp::endl;
    EV_INFO << "ScalarAnalogModelAntennaFix receptionAntennaDirection "
                   << receptionAntennaDirection << omnetpp::endl;
    double transmitterAntennaGain = transmitterAntenna->getGain()->computeGain(
            transmissionAntennaDirection);
    double receiverAntennaGain = receiverAntenna->getGain()->computeGain(
            receptionAntennaDirection);

    double pathLoss = radioMedium->getPathLoss()->computePathLoss(transmission,
            arrival);
    double obstacleLoss =
            (radioMedium->getObstacleLoss() != nullptr) ?
                    radioMedium->getObstacleLoss()->computeObstacleLoss(
                            narrowbandSignalAnalogModel->getCenterFrequency(),
                            transmission->getStartPosition(),
                            receptionStartPosition) :
                    1;
    inet::units::values::W transmissionPower =
            scalarSignalAnalogModel->getPower();
    inet::units::values::W receptionPower = transmissionPower
            * std::min(1.0,
                    transmitterAntennaGain * receiverAntennaGain * pathLoss
                            * obstacleLoss);

    EV_INFO << "ScalarAnalogModelAntennaFix transmitterAntennaGain "
                   << transmitterAntennaGain << omnetpp::endl;
    EV_INFO << "ScalarAnalogModelAntennaFix receiverAntennaGain "
                   << receiverAntennaGain << omnetpp::endl;
    EV_INFO << "ScalarAnalogModelAntennaFix pathLoss " << pathLoss
                   << omnetpp::endl;
    EV_INFO << "ScalarAnalogModelAntennaFix obstacleLoss " << obstacleLoss
                   << omnetpp::endl;
    EV_INFO << "ScalarAnalogModelAntennaFix transmissionPower "
                   << transmissionPower << omnetpp::endl;

    return receptionPower;
}

}  // namespace estnet
