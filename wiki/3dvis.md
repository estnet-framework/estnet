Hint: Further parameters can be found in OsgNode.ned, ContactPlanVisualizer.ned and OSGEarthScene.ned

**Earth Models**
```
*.osgEarthScene.sceneModel = "../data/readymap.earth"
*.osgEarthScene.sceneModel = "../data/readymap_offline8.earth"
*.osgEarthScene.sceneModel = "../data/readymap_offline.earth"
```

**Sat Models**
```
*.sat[*].osgNode.modelURL = "../data/satellite.osgb.0,-90,0.rot"
*.sat[*].osgNode.modelURL = "../data/satellite.osgb.0,-90,0.rot.100000.scale"
*.sat[*].osgNode.modelURL = "../data/UWEModel.STL.0,-90,0.rot.100.scale"
*.sat[*].osgNode.modelScale = 5000
```

**RF Satellite Cone Color**

```
*.sat[*].osgNode.satConeColor = ""
*.sat[*].osgNode.satConeColor = "#ffffff66"
```

**Ground Station Models**
```
*.cg[*].osgNode.modelURL = "../data/dishlow.osgb.0,90,0.rot"
*.cg[*].osgNode.modelURL = "../data/dishlow.osgb.0,90,0.rot.50000.scale"
```

**Coordinate Systems**

```
*.**.osgNode.coordBaseColorX = "#ff0000ff" # red
*.**.osgNode.coordBaseColorY = "#00ff00ff" # green
*.**.osgNode.coordBaseColorZ = "#0000ffff" # blue
*.**.osgNode.coordBaseLength = 500000m
*.**.osgNode.coordBaseWidth = 2.5m
```

**Coordinate System for Earth**
```
*.osgEarthScene.coordBaseColorX = "#ff0000ff" # red
*.osgEarthScene.coordBaseColorY = "#00ff00ff" # green
*.osgEarthScene.coordBaseColorZ = "#0000ffff" # blue
*.osgEarthScene.coordBaseLength = 10000000m
```

**Labels**
```
*.sat[0].label = "Chief"
```

**Line-of-sight visualization colors**

```
*.contactPlanVisualizer.satToSatColor = ""
*.contactPlanVisualizer.satToSatColorOnPlan = ""
*.contactPlanVisualizer.satToGroundColor = ""
*.contactPlanVisualizer.satToGroundColorOnPlan = ""
*.contactPlanVisualizer.satToSatColor = "#00FF00BF"
*.contactPlanVisualizer.satToSatColorOnPlan = "#00FF00BF"
*.contactPlanVisualizer.satToGroundColor = "#FFFF00BF"
*.contactPlanVisualizer.satToGroundColorOnPlan = "#FFFF00BF"
```

**Smooth Visualization**

```
**.osgNode.timeStep = 0.1s 
**.osgNode.orbitUpdateInterval = 10s
**.osgNode.orbitResolution = 1000
```

**Remove Labels, Lines and Cones**

```
*.sat[*].label = ""
*.contactPlanVisualizer.satToSatColor = ""
*.contactPlanVisualizer.satToSatColorOnPlan = ""
*.contactPlanVisualizer.satToGroundColor = ""
*.contactPlanVisualizer.satToGroundColorOnPlan = ""
```