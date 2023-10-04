#Pull the base image and update cache:
FROM ros:humble-ros-core
RUN apt update 

#Add a user:
RUN useradd -p "$(openssl passwd -1 compliant)" dingo

#Add user to sudo:
RUN usermod -aG sudo dingo

#Create directory:
RUN mkdir -p /home/dingo
Run chown dingo /home/dingo

#Install GUI required packages:
RUN apt install -y ffmpeg libsm6 libxext6

#Install project required packages:
RUN apt install -y python3-pip python3-venv nano xdotool wmctrl iputils-ping

#Copy pip files to workspace:
ADD kortex_api-2.6.0.post3-py3-none-any.whl /home/dingo
ADD requirements.txt /home/dingo
WORKDIR /home/dingo

#Install kortex wheel:
RUN pip install kortex_api-2.6.0.post3-py3-none-any.whl

#Install required pip packages:
RUN pip install -r requirements.txt

#Run Container as nonroot
USER dingo
