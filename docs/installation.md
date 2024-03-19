# Installation

## Prerequisites

### ROS
The code can be used with ROS1 Noetic and ROS2 Humble (still under development). When switching between the different ROS versions the Building Instructions described in [below](#Set-up-Docker-Container-and-Build-Workspace) have to be repeated.

### Install Docker
Docker is an application that simplifies the process of managing application processes in containers. Containers let you run your applications in resource-isolated processes. 

To install Docker on Ubuntu 22.04 follow the instructions [here](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04) and for Ubuntu 20.04 see [this](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-20-04).

###  NVIDIA Container Toolkit
If you have a Nvidia gpu, run `nvidia-smi` to check, you also need to install the NVIDIA Container Toolkit to use it in Docker.

Follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

## Set up Docker Container and Build Workspace
Clone this repo including its submodules:

```
git clone git@github.com:tud-amr/mobile-manipulator-compliance.git --recurse-submodules
```

Build the Docker container:
```bash
cd docker
bash build
```
When asked, select the full version and your ROS version.


To open the Docker container, run bellow command in /docker:
```bash
bash run
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
bash build
```


Either source workspace (<source devel/setup.bash>) or ctr+d to get out of the docker, and <bash run> in the docker folder. This autonatically sources the workspace:
```bash
cd ..
cd docker
bash run
```

Run to test if the installation was successful.
```bash
roslaunch launcher simulation.launch 
```
To close the simulation window, press esc when the window is selected. And enter ctrl + c in the terminal.