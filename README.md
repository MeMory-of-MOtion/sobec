# Sobec: Sandbox for optimal control explicitly for bipeds

[![Pipeline status](https://gitlab.laas.fr/memory-of-motion/sobec/badges/master/pipeline.svg)](https://gitlab.laas.fr/memory-of-motion/sobec/commits/master)
[![Coverage report](https://gitlab.laas.fr/memory-of-motion/sobec/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/memory-of-motion/sobec/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Sandbox for optimal control explicitly for bipeds

Minimal python version: 3.6

## Current state of the MPC, Classes:

#### RobotDesigner
This is a robot wrapper embedding the model

#### ModelMaker
This class produces a `std::vector` of `AbstractModelAction`, it is done using the method `formulateHorizon`.

The class contains all task functions used to formulate a whole-body locomotion problem.

#### WBC
This class implements a MPC with a cycle view of locomotion.

It defines three horizon objects: the MPC horizon, the walking cycle horizon and the standing cycle horizon.
At each control cycle, the first walking cycle horizon node is fed to the end of the MPC horizon. When the walking cycle comes to an end, it is rewinded.
When the user wants to stop the locomotion, they can do so at any time by simply updating the LocomotionType variable from WALKING to STANDING.
The standing cycle will then be fed to the MPC horizon.

The WBC class provides the method `iterate` that receives the measured state and returns the joint torques that should be commanded to the robot.

#### WBCHorizon
This class implements a MPC with a longterm view of horizon.

It defines two horizon objects: the MPC horizon and the full horizon.
At each control cycle, the first node of the full horizon is fed to the end of the MPC horizon. When the full horizon comes to an end, the process stops.
The class previews the entire walking motion into the full horizon at its initialization. It also sets up the wrench references for contact transition at initialization.

#### Examples

Python examples running with bullet can be found in 'python/tests folder.

`walkMPC.py` implements a forward walking motion with user-defined feet references.

`freeWalkMpc.py` implements a forward walking motion with user-defined terminal CoM position.

`stairMPC.py` implements stair climbing with user-defined feet references.

`freeStairsMPC.py` implements stair climbing without user-defined feet references, given only the next desired contact and a velocity height map of the stairs.
