# Villa Simulation

Contains Gazebo worlds, models for RoboCup@Home tasks.

# Worlds

## Stage 1 Task 1: Storing Groceries

The robot starts facing a cupboard containing a handful of items. The cupboard has a single door, which is closed initially. There are 10 objects on a nearby table. The robot must move all of the objects from the table to the cupboard, grouping related items. (5.1)

# Usage

These are just assets. To run simulations, please see the launch files in [villa_launch](https://github.com/AustinVillaatHome/villa/tree/master/villa_launch).

# Development

Gazebo 2 does not have a model editor. One alternative is to modify files using a text editor, then view the result in Gazebo by placing an instance. In order to have this repo's models appear in the Gazebo insert interface, you'll need put the correct folder onto the model path before launching.

	export GAZEBO_MODEL_PATH=${HOME}/robocup_home_ws/src/villa_simulation/models:${GAZEBO_MODEL_PATH}

