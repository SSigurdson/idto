#!/usr/bin/env python

##
#
# Use drake's ModelVisualizer to view the Franka + Allegro Hand + Pen system. 
# This will help us determine reasonable initial conditions and target poses.
#
##

from pydrake.all import ModelVisualizer, StartMeshcat

meshcat = StartMeshcat()
visualizer = ModelVisualizer(meshcat=meshcat)

visualizer.AddModels("examples/models/fr3_algr_minimal_collision.urdf")
visualizer.AddModels("examples/models/pen.sdf")
visualizer.Finalize()

# Set initial positions
q =  [0.0, 0.6, 0.1,-1.8, 1.6, 1.7, 0.0,  # arm joint angles
      0.0, 0.7, 0.7, 0.6,                 # index finger
      0.0, 0.7, 0.7, 0.6,                 # middle finger
      0.0, 1.4, 0.9, 0.6,                 # ring finger
      1.4, 0.2, 0.7, 0.6,                 # thumb
      1.0, 0.0, 0.0, 0.0,                 # ball orientation
      0.79, 0.27, 0.2]                     # ball position
visualizer._sliders.SetPositions(q)

visualizer.Run()