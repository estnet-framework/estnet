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

package estnet.node.tracking.swipe;
import estnet.node.tracking.contract.INodeTracking;

simple ContactPlanLessSwipeTracking like INodeTracking
{
    double updateInterval @unit(s) = default(0.1s);
	bool enable = default(true);
	string gsContactPeriods = default("0 1");
	double trackSinAmount = default(0.0);
}
