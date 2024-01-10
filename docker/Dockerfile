#Pull the base image and update cache:
ARG ROS
FROM ros:${ROS}
RUN apt update 

#Add a user:
RUN useradd -p "$(openssl passwd -1 compliant)" dingo

#Add user to sudo:
RUN usermod -aG sudo dingo

#Create directory:
RUN mkdir -p /home/dingo/compliant-control
Run chown dingo /home/dingo

#Install project required packages:
RUN apt install -y git python3-pip libboost-all-dev iputils-ping nano
ARG ROS

#For noetic, install curl for fix (see later) and install dependencies for vicon bridge:
RUN if [ "$ROS" = "noetic" ]; then \
        apt install -y curl ros-noetic-tf ros-noetic-diagnostic-updater; \
    fi

#Install GUI related packages:
ARG INSTALL
RUN if [ "$INSTALL" = "full" ]; then \
        apt install -y ffmpeg libsm6 libxext6 xdotool wmctrl ros-$ROS_DISTRO-plotjuggler-ros libxcb-cursor0; \
    fi

#Fix for noetic: https://stackoverflow.com/questions/49478573/pip3-install-not-working-no-module-named-pip-vendor-pkg-resources
ARG ROS
RUN if [ "$ROS" = "noetic" ]; then \
        curl -sS https://bootstrap.pypa.io/get-pip.py | sudo python3 \
        && type pip3 \
        && hash -r; \
    fi

#Copy files to workspace:
ADD python /home/dingo/compliant-control/python
ADD docker/.bashrc /home/dingo
ADD docker/kortex_api-2.6.0.post3-py3-none-any.whl /home/dingo
ADD docker/requirements_full.txt /home/dingo
ADD docker/requirements_core.txt /home/dingo
ADD docker/requirements_flake8.txt /home/dingo
WORKDIR /home/dingo

#Install kortex wheel:
RUN pip install kortex_api-2.6.0.post3-py3-none-any.whl

#Install python directory:
RUN pip install -e /home/dingo/compliant-control/python

#Install required pip packages:
ARG INSTALL
RUN if [ "$INSTALL" = "full" ]; then \
        pip install -r requirements_full.txt \
        && pip install -r requirements_flake8.txt; \
    else \
        pip install -r requirements_core.txt; \
    fi

#Generate stubs:
RUN pip install mypy
WORKDIR /usr/local/lib/python3.10/dist-packages/mujoco
RUN stubgen -p mujoco -o ../.

#Run ontainer as nonroot
USER dingo
WORKDIR /home/dingo/compliant-control
