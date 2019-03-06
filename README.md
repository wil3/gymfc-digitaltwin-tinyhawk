# gymfc-digitaltwin-tinyhawk

TODO: Convert readme into a better format. Currently it's just a list of observations.

- Don't use 3D acceleration for your VM when running Gazebo's UI -> viewport will be black, but Gazebo will still run. Might work if running Gazebo in headless mode.
- Fusion360 apparently spits out the STL models in the exact position they were in inside the assembly.
- Unfortunately, Fusion360's Y axis is not the same Y axis in Gazebo. Rotate imported models around the X axis 2 snaps while holding CTRL to re-orient the model.