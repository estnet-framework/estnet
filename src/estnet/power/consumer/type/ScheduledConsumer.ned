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

package estnet.power.consumer.type;

import estnet.power.base.ConsumerStateHandler;

// A detailed schedule is given for the activity of the
// system by periodical consumption values in a csv file
// The module requires a path to the csv file, the name of
// the column in which the consumption values are listed, which
// enables to list the values for all satellites in one csv
// and just choose by the column name
// eg: 
//	*.sat[*].energyModule.consumer.OCS.stateHandler1.columnName = "schedule.csv" 
//	*.sat[*].energyModule.consumer.OCS.stateHandler1.csvTimestep = 10s 
//	*.sat[0].energyModule.consumer.OCS.stateHandler1.columnName = "C_sat0" 
//	*.sat[1].energyModule.consumer.OCS.stateHandler1.columnName = "C_sat1" 
//	*.sat[2].energyModule.consumer.OCS.stateHandler1.columnName = "C_sat2" 
module ScheduledConsumer extends ConsumerStateHandler
{
    parameters:
        double csvTimestep @unit(s); // time steps between the .csv entries
        string fileName;	// file name of .csv
        string columnName;	// name of the coloumn of the consumption entries (eg. "Cons_Sat1")
        @class(ScheduledConsumer);
}
