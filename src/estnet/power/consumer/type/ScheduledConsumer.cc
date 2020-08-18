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

#include "ScheduledConsumer.h"

using namespace estnet;

Define_Module(ScheduledConsumer);

ScheduledConsumer::~ScheduledConsumer() {
    cancelAndDelete(this->_updateTimer);
}

void ScheduledConsumer::initialize(int stage) {
    if (stage == inet::InitStages::INITSTAGE_PHYSICAL_OBJECT_CACHE) {

        _timestep = par("csvTimestep").doubleValueInUnit("s");
        //read csv data and save as schedule
        std::ifstream csvFile = std::ifstream(par("fileName").stringValue());
        if (!csvFile.is_open())
            EV_ERROR << "Consumption schedule not found" << omnetpp::endl;

        char sep = ';';
        // the first line has seperator information
        if (csvFile.good()) {
            std::string line;
            std::getline(csvFile, line);
            if (line.find(';') != std::string::npos) {
                sep = ';';
            } else if (line.find(',') != std::string::npos) {
                sep = ',';
            }
        }
        std::string *titles = new std::string[100];
        int numTitles = 0;
        // read the first line containing the title of each cell
        if (csvFile.good()) {
            std::string line;

            std::getline(csvFile, line);
            std::stringstream lineStream(line);

            while (std::getline(lineStream, titles[numTitles], sep)) {
                numTitles++;
            }
        }

        int csvColumn;
        const char *columnName = par("columnName").stringValue();
        for (int i = 0; i < numTitles; i++) {
            if (titles[i].find(columnName) != std::string::npos) {
                csvColumn = i;
                break;
            }
        }
        delete[] titles;
        std::string timeStr;
        std::string consumptionStr;
        double time = 0;
        int counter = 0;
        while (csvFile.good()) {
            std::string line;
            std::string *tokens = new std::string[100];
            std::getline(csvFile, line);
            std::stringstream lineStream(line);
            int i = 0;
            while (std::getline(lineStream, tokens[i], sep)) {
                i++;
            }
            if (tokens[csvColumn].empty()) {
                delete[] tokens;
                break;
            }
            ConsumptionProperties cons;
            double c = std::stod(tokens[csvColumn]);

            cons.time = time;
            cons.consumption = W(c);
            cons.on = c > 0;

            this->_scheduledConsumption.push_back(cons);
            time += _timestep;
            counter++;
        }
        csvFile.close();

        //initialize with first consumption value and schedule the next change
        this->_updateTimer = new cMessage("update power consumption");
        this->_currentSchedule = this->_scheduledConsumption.begin();
        scheduleAt(SimTime(this->_currentSchedule->time, SIMTIME_S),
                this->_updateTimer);

        //get consumer module
        const char *consumerModule = par("consumerModule");
        this->_energyConsumer = check_and_cast<ConsumerModuleBase*>(
                getModuleByPath(consumerModule));

    } else if (stage == 5)
        _energyConsumer->addStateHandler(this);

}

void ScheduledConsumer::handleMessage(cMessage *msg) {
    if (msg->isSelfMessage()) {
        //get next scheduled power consumption planned for the time, when the timer was scheduled
        this->updatePowerConsumption();
        //schedule the next change
        if (this->_currentSchedule != this->_scheduledConsumption.end())
            scheduleAt(SimTime(this->_currentSchedule->time, SIMTIME_S), msg);
    }
}

void ScheduledConsumer::updatePowerConsumption() {
    //switch the state
    this->_on = this->_currentSchedule->on;
    //get next scheduled consumption and publish it
    this->_powerConsumption = this->_currentSchedule->consumption;
    EV_DEBUG << "Publishing a scheduled consume of " << this->_powerConsumption
                    << std::endl;
    this->_energyConsumer->powerConsumptionChanged();
    //increment the iterator for next consumption
    this->_currentSchedule++;
}

