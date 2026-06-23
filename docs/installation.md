# Installation

## Prerequisites

### ROS
The code can be used with ROS1 Noetic.

## Set up Workspace and Build Workspace
Clone this repo including its submodules:

```
git clone git@github.com:tud-amr/mobile-manipulator-compliance.git --recurse-submodules
```

Since we use the same image for simulation and the real robot, we also need to build the Dingo driver. To build dingo-driver run: 
```bash
cd ..
cd cpp
bash build
```

We make use of symbolic definitions for e.g. the gravitational vector. To build them run:
```bash
cd ..
cd python/compliance_control/control/symbolics
bash compile
```

Build ros workspace:
```bash
cd ../../../../
cd ros
catkin build
source devel/setup.bash
```
