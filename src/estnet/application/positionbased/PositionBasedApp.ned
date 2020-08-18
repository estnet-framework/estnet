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

package estnet.application.positionbased;

import estnet.application.base.BasicApp;

//
// An app, thats generates data based on the current position of the node
// and a distribution of such positions around the globe.
//
simple PositionBasedApp extends BasicApp
{
    parameters:
        volatile double basicDataInterval = default(1);					// Packets are send with a time delay of basicDataInterval/multiplier
        volatile double idleInterval @unit(s) = default(1s);			// Interval for checks if new data is available if multiplier is 0
        volatile double beamWidth @units(deg) = default(20);			// Beamwidth of the antenna used to receive data from ground; is ignored if maxFOV = ture
        volatile bool maxFOV = default(true);							// Uses the maximal possible FOV of the satellite as beam width
        volatile string dataSource = default("49.786844, 9.980713");	// Selects the data source to get data from; intial value is Wuerzburg; possible options: "lat, long" or "AIS" or "Memorized"
        volatile string dataPath = default("");							// Path to the file from which the data should be loaded
        volatile bool useNormalOfMultiplier = default(true);			// Selects whether a normal distibution is used over the multiplier given by the data
        @class(PositionBasedApp);
}
