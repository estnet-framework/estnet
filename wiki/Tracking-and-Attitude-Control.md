The different tracking options can be used to point either a groundstation or a satellite at a specific target to e.g. improve communication with a directional antenna or increase the generated power py solar cells. Furthermore, they are able to consider limitations in how fast a desired orientation can be reached.

## GS Tracking
The tracking of a target with a groundstation is provided over the INodeTracking interface, where already some implementations are available.

### Target Tracking
To track a single node with the groundstation the TargetTracking option can be used. To set this option in the .ini file use:
```
*.cg[*].trackingType = "TargetTracking"
*.cg[*].nodeTracking.target = "<target>"
```
To define a specific target it uses the AttitudeTarget class. A unique target can be set using one of the following options:

| option | description |
| ------ | ------ |
| nodeNumber | The specific number of the node that should be tracked. Note, counting starts at 1 with the satellites coming first and the groundstation afterwards. |
| EARTH_CENTER | The center of the earth. |
| SUN | The position of the sun. |
| NIL | No object is tracked. |
| x, y, z | 3D coordinates of a point in the ECI frame in meters. |

### Mean Tracking
To track multiple satellites, which are in close proximity, MeanTracking can be used. It computes the weighted average of the position of multiple nodes and tracks it. One must give the node id of the first (`startNodeId`) and the last (`stopNodeId`) contact of the formation. 
Additionally, a contact plan must be provided to enable the scheduling of ground station passes. To enable it use this in the .ini file:
```
*.cg[*].trackingType = "MeanTracking"
*.cg[*].nodeTracking.startNodeId = <first satellite>
*.cg[*].nodeTracking.stopNodeId = <last satellite>
```
The weighting can be adjusted with the `exponent` option, which is 1 by default. It uses exponential weighting where `exponent` is the exponent.

### Contactplan based Tracking
Track always the current target based on the provided contactplan. Make sure to set `*.contactPlanManager.contactPlanFile = "<contactplanFile>"`. This is the default option, so it is not necessary to set it manually.
```
*.cg[*].trackingType = "ContactPlanBasedNodeTracking"
```
### Weighted Swipe Tracking
The idea of a weighted swipe tracking is to give each node a weight, that determines how much time of a groundstation pass is used to track this node. One must give the node id of the first (`startNodeId`) and the last (`stopNodeId`) contact of the formation.
Additionally, a contact plan must be provided to enable the scheduling of ground station passes.
```
*.cg[*].trackingType = "WeightedSwipeTracking"
*.cg[*].nodeTracking.startNodeId = <first satellite>
*.cg[*].nodeTracking.stopNodeId = <last satellite>
```
### Exponential Swipe Tracking
The exponential swipe tracking is used to swipe through a along-track formation according to a given function.
One must give the node id of the first (`startNodeId`) and the last (`stopNodeId`) contact of the formation.
Additionally, a contact plan must be provided to enable the scheduling of ground station passes.
```
*.cg[*].trackingType = "ExponentialSwipeTracking"
*.cg[*].nodeTracking.startNodeId = <first satellite>
*.cg[*].nodeTracking.stopNodeId = <last satellite>
*.cg[*].nodeTracking.exponent = <exponent>
```
### Contactplanless Swipe Tracking
With this tracking, one can track the satellites of a string-of-perls like formation without contact plan. Therefore, the pass time must be given in `gsContactPeriods`. The tracking is done in a sinusoidal way. 
```
*.cg[*].trackingType = "ContactPlanLessSwipeTracking"
*.cg[*].nodeTracking.trackSinAmount = 0.2
*.cg[*].nodeTracking.gsContactPeriods = "700 1300" # contact periods in simtime [s]
```
### Limiting Movement Speed
By default, the groundstation can rotate arbitrary fast and always points accurately at its target. For a more realistic simulation the motion can be limited in terms of rotation speed on both the elevation and the azimuth axis. To enable the simulation of this set this in your .ini file:
```
*.cg[*].networkHost.mobility.enableKinematics = true
```
The rates can adjusted with:
```
*.cg[*].networkHost.mobility.maxAzimuthRate = <max rotation speed in rad/sec>
*.cg[*].networkHost.mobility.maxElevationRate = <max rotation speed in rad/sec>
```

## Satellite Tracking
The tracking of a target with a satellite is achieved with the AttitudeController. It models the natural motion of a satellite in space and takes the limited options of movement into consideration.

The target for a satellite is specified in the AttitudeController with the AttitudeTarget class using the following .ini option:
```
*.sat[*].attitudeController.target = "<target>"
```
The options for \<target> are:

| option | description |
| ------ | ------ |
| nodeNumber | The specific number of the node that should be tracked. Note, counting starts at 1 with the satellites coming first and the groundstation afterwards. |
| EARTH_CENTER | The center of the earth. |
| SUN | The position of the sun. |
| NIL | No object is tracked. |
| x, y, z | 3D coordinates of a point in the ECI frame in meters. |

To limit the satellites motion on how fast it can turn to a target, the maximum possible angular acceleration in radiants per second squared can be set either as single value equal in every axis:
```
*.sat[*].attitudeController.angularAcceleration = "0.12"
```
or on a per axis basis:
```
*.sat[*].attitudeController.angularAcceleration = "0.12, 0.024, 0.024"
```
The last option is especially useful, if the satellite has asymmetries.