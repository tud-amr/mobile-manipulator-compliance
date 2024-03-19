# Kinova Compliant Control

This package contains the software to control the Kinova Gen3 Lite using impedance control.


## Kinova's estimated current/torque ratios.

Kinova provides current feedback and an estimated torque feedback. Measurements show they used these current/torque ratios:


Joint: | Size: | Ratio:
-------|-------|-------
| 0 | M | 0.85
| 1 | L | 0.25
| 2 | M | 0.85
| 3 | S | 1.75
| 4 | S | 1.75
| 5 | S | 1.75

## Installation steps:
Check for nvidia-gpu:
nvidia-smi
sudo docker run --rm --gpus all ubuntu nvidia-smi
git clone ... --recurse-submodules  (???)
If you already cloned it without submodules, you have to do this:
git submodule update --init

To pull newer version (which already should have the submodules downloaded via the method above): git pull --recurse-submodules

Install docker:
-- add links

Go to the folder, compliance control, docker. BUILD DOCKER:
``` bash
bash build
```
To open the docker container:
```
bash run
```

So we are missing the dingo-driver, which is not required, but included. To build dingo-driver (cpp dependencies): 
```bash
cd cpp
bash build
```

You still need to build some c++ files that are needed for symbolic definitions:
```bash
cd python/compliance_control/control/symbolics
bash compile
```

Build ros container, and run simulation on laptop from ros-folder:
```bash
cd ros
bash build
```


either source workspace (<source devel/setup.zsh>) or ctr+d to get out of the docker, and <bash run> in the docker folder:
```bash
cd docker
bash run
roslaunch launcher simulation.launch #to test if it works, start the simulation
```

## INTERACTING WITH THE SIMULATION
To start the simulation (if not done already)
```bash
roslaunch launcher simulation.launch
```
To escale simulation, <esc>.

1. Click refresh at first, to send message to the robot to get the state. 
2. Press <Start-llc>, press <Start-llc-task>, now it is in current mode, and it is only compensating gravity. To stop <stop llc>
3. To interact with the robot, <ctr, right-click mouse> and drag the robot to a desired pose.  
4. To stop <stop llc-task>, <stop llc>
5. To go to a desired position again: press <pref> and select a position that you would like.


Other comments:
- The keys for Mujoco, can be found online. If you do <tab> you get all the infopannels, or <f1> to get an overview of some commands that you can do. 
- If you would like to remove the toolbox automatically from your screen: settings, appearance, auto-hide the dock.
- If you would like to calibrate, press <calibrate>, but it will take some time. 






