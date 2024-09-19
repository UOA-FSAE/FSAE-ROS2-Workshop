FROM osrf/ros:humble-desktop
LABEL Name=dev_ws Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /

RUN apt-get update 
RUN apt-get upgrade -y

# install python
RUN apt-get install software-properties-common -y
RUN add-apt-repository ppa:deadsnakes/ppa -y
RUN apt update
RUN apt install python3.8 -y
RUN echo "alias python=/usr/bin/python3.8" >> ~/.bashrc
RUN echo "alias python3=/usr/bin/python3.8" >> ~/.bashrc

# install ROS2
RUN apt-get install --no-install-recommends -y \
    python3-pip \
    ros-humble-foxglove-bridge
RUN apt-get clean

# remove easyinstall deprecation warning
RUN pip3 install setuptools==58.2.0

ENV ROS_DISTRO humble

# RUN rosdep update --rosdistro $ROS_DISTRO 
# RUN apt-get update 
# RUN rosdep install --from-paths src -y -r --ignore-src --rosdistro=$ROS_DISTRO --os=ubuntu:jammy
# RUN rm -rf /var/lib/apt/lists/*
# RUN apt-get clean

# RUN colcon build --parallel-workers $(nproc) --symlink-install \
#         --event-handlers console_direct+ --base-paths src \
#         --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
#         ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
#         ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "alias sros='source /devs_ws/install/setup.bash'" >> ~/.bashrc

RUN apt-get install openssh-server -y
RUN sed -E 's/^#(PermitRootLogin )prohibit-password/\1yes/' /etc/ssh/sshd_config -i
RUN mkdir /run/sshd
RUN ssh-keygen -A

ARG WS_PSWD

RUN echo -e "${WS_PSWD}\n${WS_PSWD}" | passwd

RUN mv /lib/systemd/system/ssh.service /etc/systemd/system/sshd.service

# set with:             `docker build --build-arg ROS_DOMAIN_ID=<ROS_DOMAIN_ID>`
# overwriten with:      `docker run -e ROS_DOMAIN_ID=<ROS_DOMAIN_ID>`
ARG ROS_DOMAIN_ID=47 
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID} 

# set with:             `docker build --build-arg TURTLE_NAME=<TURTLE_NAME>`
# overwriten with:      `docker run -e TURTLE_NAME=<TURTLE_NAME>`
ARG TURTLE_NAME=Turtle1 
ENV TURTLE_NAME=$DEFAULT_TURTLE_NAME

RUN mkdir /devs_ws
COPY ws /devs_ws
# ADD https://github.com/UOA-FSAE/FSAE-ROS2-Workshop-Workspace /devs_ws
# RUN git clone <WORKSHOP_REPO>

WORKDIR /devs_ws

CMD ["/usr/sbin/sshd", "-D"]