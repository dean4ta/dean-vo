FROM osrf/ros:melodic-desktop-full
SHELL ["/bin/bash","-c"] 

ENV ROS1_DISTRO melodic

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  build-essential \
  python-rosdep \
  && rm -rf /var/lib/apt/lists/*

# Setup Locales
RUN apt-get update && apt-get install -y locales
ENV LANG="en_US.UTF-8" LC_ALL="en_US.UTF-8" LANGUAGE="en_US.UTF-8"

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
   ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
  ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
  locale-gen --purge $LANG && \
  dpkg-reconfigure --frontend=noninteractive locales && \
  update-locale LANG=$LANG LC_ALL=$LC_ALL LANGUAGE=$LANGUAGE

# Set up timezone
ENV TZ 'America/Los_Angeles'
RUN echo $TZ > /etc/timezone && \
  rm /etc/localtime && \
  ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && \
  dpkg-reconfigure -f noninteractive tzdata

# Install basic dev and utility tools
RUN apt-get update && apt-get install -y \
  apt-utils \
  git \
  lsb-release \
  build-essential \
  stow \
  neovim \
  nano \
  tmux \
  wget \
  htop \
  unzip \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  x11-apps

# create catkin directories
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src/
WORKDIR ${CATKIN_WS}/src

COPY . .

WORKDIR ${CATKIN_WS}
RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
  # Install dependencies
  && apt-get update && apt-get install -y \
  && rosdep install -y --from-paths . --ignore-src -r --rosdistro ${ROS1_DISTRO}

RUN source /opt/ros/${ROS1_DISTRO}/setup.bash \
  # Build catkin workspace
  && catkin_make
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
