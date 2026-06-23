# Application on the Robot

## Connect to Robot
Follow the instructions provided [here](https://www.clearpathrobotics.com/assets/guides/melodic/dingo/network.html) to connect to the Dingo.

## Setup
Clone the repository including its submodules on the robot and follow the steps as provided [here](/docs/installation.md#set-up-docker-container-and-build-workspace). This time select the core version.

## Mobile Manipulator Control
### Installation without docker on the robot:
```bash
# clone repo, make python package
git clone git@github.com:INTERACT-tud-amr/mobile-manipulator-compliance.git --recurse-submodules
pip install -e python

#required sudo dependencies!!!!
sudo apt install -y git python3-pip libboost-all-dev iputils-ping nano
sudo apt-get update
sudo apt install -y curl ros-noetic-tf ros-noetic-diagnostic-updater

# install python dependencies:
cd docker
pip install -r requirements_core.txt

# Build cpp package
cd cpp
bash build

# symbolics compilation:
cd python/compliance_control/control/symbolics
bash compile

# ros:
cd ros/noetic
catkin_make #local catkin_make
cd ../..
catkin_make #global catkin_make
```

### Turn off default ros packages by clearpath
On robot outside the docker run
```bash
sudo systemctl stop ros.service
```
The lights on the front of the robot will become red.


### Run controller
Run the controller:
```bash
roslaunch launcher dinova_bringup_compliant.launch
# Before doing ctrl+c, STOP the compliant mode, press the cross button on the dingo-joystick
```
For more commands, see [here](/docs/helpful_commands.md)

