FROM osrf/ros:humble-desktop
LABEL Name=dev_ws Version=0.0.1

SHELL [ "/bin/bash", "-c" ]

WORKDIR /

RUN apt-get update 
RUN apt-get upgrade -y
RUN apt-get install --no-install-recommends -y \
    python3-pip \
    ros-humble-foxglove-bridge
RUN apt-get clean

ENV ROS_DISTRO humble

RUN mkdir /devs_ws
COPY ws /devs_ws

RUN source /opt/ros/humble/setup.bash
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


RUN cd /devs_ws
# TODO
# RUN git clone <WORKSHOP_REPO>

RUN apt-get install openssh-server -y
RUN sed -E 's/^#(PermitRootLogin )prohibit-password/\1yes/' /etc/ssh/sshd_config -i
RUN mkdir /run/sshd
RUN ssh-keygen -A

ENV build_var=

RUN echo -e "${WS_PSWD}\n${WS_PSWD}" | passwd

RUN mv /lib/systemd/system/ssh.service /etc/systemd/system/sshd.service

CMD ["/usr/sbin/sshd", "-D"]