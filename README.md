# Sobec: Sandbox for optimal control explicitly for bipeds

[![Pipeline status](https://gitlab.laas.fr/memory-of-motion/sobec/badges/master/pipeline.svg)](https://gitlab.laas.fr/memory-of-motion/sobec/commits/master)
[![Coverage report](https://gitlab.laas.fr/memory-of-motion/sobec/badges/master/coverage.svg?job=doc-coverage)](http://projects.laas.fr/gepetto/doc/memory-of-motion/sobec/master/coverage/)
[![Code style: black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

Sandbox for optimal control explicitly for bipeds

Minimal python version: 3.6

## Current state of the MPC, Classes:

#### RobotDesigner
it is a robot wrapper (After, it will be called RobotWrapper)

#### ModelMaker
This class produces a `std::vector` of `AbstractModelAction`, it is done using the method `formulateHorizon`.

There is only one formulation implemented right now, specified on the method `formulateStepTracker`, it would be good to incorporate the other formulations in new methods such as `formulateStairClimber` or `formulateWithoutThinking`. Reuse the contacts and costs methods that are already made (with names starting by `define...`).

Once new formulations are made, it would be good to have a name based selector in the method `formulateHorizon`.

#### HorizonManager
it is the OCP (After, it will be called OCP)

It receives a vector of `AbstractModelActions` (created by the ModelMaker) and provides methods to deal with the `ddp` object of crocoddyl.

#### WBC
it is the MPC (After it will be called MPC)

It provides the method iterate that receives the measured state and returns the joint torques that should be commanded in the robot.
All previous classes are used here.

#### MainControlLoop
It is missing, this script should instantiate the WBC and computes the control in a loop with ros.
