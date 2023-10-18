#Pull the base image and update cache:
FROM ros:humble
RUN apt update 

#Add a user:
RUN useradd -p "$(openssl passwd -1 compliant)" dingo

#Add user to sudo:
RUN usermod -aG sudo dingo

#Create directory:
RUN mkdir -p /home/dingo/kinova_compliant_control
Run chown dingo /home/dingo

#Install GUI related packages:
RUN apt install -y ffmpeg libsm6 libxext6 xdotool wmctrl

#Install project required packages:
RUN apt install -y git python3-pip libboost-all-dev iputils-ping nano

#Copy files to workspace:
ADD python /home/dingo/kinova_compliant_control/python
ADD docker/.bashrc /home/dingo
ADD docker/kortex_api-2.6.0.post3-py3-none-any.whl /home/dingo
ADD docker/requirements.txt /home/dingo
ADD docker/requirements_gui.txt /home/dingo
ADD docker/requirements_flake8.txt /home/dingo
WORKDIR /home/dingo

#Install kortex wheel:
RUN pip install kortex_api-2.6.0.post3-py3-none-any.whl

#Install python directory:
RUN pip install -e /home/dingo/kinova_compliant_control/python

#Install required pip packages:
RUN pip install -r requirements.txt
RUN pip install -r requirements_gui.txt

#Install flake8:
RUN pip install -r requirements_flake8.txt

#Generate stubs:
RUN pip install mypy
WORKDIR /usr/local/lib/python3.10/dist-packages/mujoco
RUN stubgen -p mujoco -o ../.

#Run ontainer as nonroot
USER dingo
WORKDIR /home/dingo/kinova_compliant_control
