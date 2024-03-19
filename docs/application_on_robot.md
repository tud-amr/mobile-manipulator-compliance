# Application on the Robot

## Connect to Robot
Follow the instructions provided [here](https://www.clearpathrobotics.com/assets/guides/melodic/dingo/network.html) to connect to the Dingo.

## Setup
Clone the repository including its submodules on the robot and follow the steps as provided [here](/docs/installation.md#set-up-docker-container-and-build-workspace). This time select the core version.

## Mobile Manipulator Control
### Turn off default ros packages by clearpath
On robot outside the docker run
```bash
sudo systemctl stop ros.service
```
The lights on the front of the robot will become red.


### Run controller
Adapt the export file to have the IP address of your robot. 

On the robot start the docker:
```bash
cd docker
bash run
```
and run the controller:
```bash
. export
roslaunch launcher robot_control_interface.launch
```
On your laptop also start the docker and run:
```bash
. export
roslaunch launcher robot_user_interface.launch
```
This will run the guidance mode. To run the tracking mode we make use of an optical tracking system. 

When switching between the control modes restart the user interface.



### Interacting with the visualization tool
Start LLC as described in the simulation section. Turn on gravity compensation and friction compensation. Then select `<arm>` starts the impedance controller on the arm, `<null>` to turn on the nullspace controller, and `<base>` to turn on the base.

Top stop turn off LLC `<Stop LCC Task>`, `<Stop LCC>` and go into the preffered position `<Pref>` then esc. It is preffered to turn the robot off in high-leve control.




## Helpful Information
If the arm reaches a joint limit, `<stop LLC Task?` and `<stop LLC?` and then press `<clear?`.

