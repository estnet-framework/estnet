Make sure the program `osgearth_package` is installed:
* On Ubuntu it is installed with the package `osgearth`.
* On Windows it needs to be built from the source: http://docs.osgearth.org/en/latest/startup.html

Then run:
`osgearth_package --tms <path to SatKommSim>/src/OSG/Data/readymap.earth --out <empty directory where to store offline maps> --out-earth <.earth file name>.earth --max-level <level of detail>`

The small default offline maps in the repository were created with: `osgearth_package --tms SatKommSim/src/OSG/Data/readymap.earth --out SatKommSim/src/OSG/Data/ --out-earth readymap_offline.earth --max-level 3`

If the `--out` directory already contains offline maps, the command might get stuck. In that case remove `readymap_elevation`, `readymap_imagery` and `readymap_offline.earth` and run the command again.