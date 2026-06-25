# Stage 1 — builder
# Same base image as the runtime stage to guarantee identical ABI for Gazebo plugins.
# Shared objects compiled here load against the same gz-harmonic, so there is no symbol mismatch.

FROM osrf/ros:jazzy-desktop AS builder

ARG PX4_VERSION=v1.17.0
ARG DEBIAN_FRONTEND=noninteractive

# OSRF repository needed for gz-harmonic at compile time
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl lsb-release gnupg ca-certificates git \
    && curl https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
       signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable \
       $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# Clone PX4 with all submodules; shallow clone reduces network traffic
RUN git clone --recursive --depth 1 --branch ${PX4_VERSION} \
    https://github.com/PX4/PX4-Autopilot.git /px4_src

# ubuntu.sh installs the correct build dependencies for the cloned version
# RUNS_IN_DOCKER=true  makes the script install sudo before using it
# USER=root            prevents usermod from failing with an empty variable
# --no-nuttx           skips the hardware toolchain; only SITL is needed here
RUN cd /px4_src \
    && RUNS_IN_DOCKER=true USER=root DEBIAN_FRONTEND=noninteractive \
       bash Tools/setup/ubuntu.sh --no-nuttx

# Build the SITL target
RUN cd /px4_src && make -j$(nproc) px4_sitl_default

# Collect only the runtime artifacts into a staging directory.
# cp -rL resolves symlinks so the final COPY does not encounter broken links.
RUN set -e; \
    B=/px4_src/build/px4_sitl_default; \
    T=/px4_rt; \
    \
    mkdir -p $T/build/px4_sitl_default; \
    cp -rL $B/bin    $T/build/px4_sitl_default/; \
    cp -rL $B/etc    $T/build/px4_sitl_default/; \
    cp -rL $B/rootfs $T/build/px4_sitl_default/; \
    \
    mkdir -p $T/build/px4_sitl_default/src/modules/simulation; \
    cp -rL $B/src/modules/simulation/gz_plugins \
           $T/build/px4_sitl_default/src/modules/simulation/; \
    \
    mkdir -p $T/Tools/simulation/gz; \
    cp -rL /px4_src/Tools/simulation/gz/models $T/Tools/simulation/gz/; \
    cp -rL /px4_src/Tools/simulation/gz/worlds $T/Tools/simulation/gz/; \
    \
    mkdir -p $T/Tools/simulation/gazebo-classic/sitl_gazebo-classic; \
    cp -rL /px4_src/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models \
           $T/Tools/simulation/gazebo-classic/sitl_gazebo-classic/; \
    \
    mkdir -p $T/src/modules/simulation/gz_bridge; \
    cp /px4_src/src/modules/simulation/gz_bridge/server.config \
       $T/src/modules/simulation/gz_bridge/


# Stage 2 — runtime
# Final image: ROS 2 Jazzy + Gazebo Harmonic + PX4 artifacts from the builder + QGroundControl.

FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# PX4_DIR points to the tree copied from the builder; the launch file reads this variable
ENV PX4_DIR=/px4

# OSRF repository needed for gz-harmonic and its Python bindings
# osrf/ros:jazzy-desktop ships only the ROS repository, not the Gazebo one
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl lsb-release gnupg ca-certificates \
    && curl https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
       signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable \
       $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# System dependencies
#   ros-jazzy-ros-gz-*       Gazebo to ROS 2 bridges
#   gz-harmonic              Gazebo Harmonic (pulls in python3-gz-transport13 and
#                            python3-gz-msgs10 as transitive dependencies)
#   python3-gz-transport13   Python gz-transport bindings (wind_turbulence, RTF stats)
#   python3-gz-msgs10        Python gz-msgs bindings
#   python3-psutil           Per-process CPU and RAM monitoring (validation_node)
#   sysstat / htop / procps  pidstat and process monitors
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-jinja2 \
    python3-yaml \
    python3-psutil \
    python3-gz-transport13 \
    python3-gz-msgs10 \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    gz-harmonic \
    sysstat \
    htop \
    procps \
    mesa-vulkan-drivers \
    vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# Enable sysstat data collection
RUN sed -i 's/ENABLED="false"/ENABLED="true"/' /etc/default/sysstat || true

# Extra Python packages
RUN pip3 install --no-cache-dir --break-system-packages \
    jinja2 \
    pyyaml \
    psutil

# MicroXRCE-DDS agent: bridges PX4 uORB to ROS 2 DDS over RTPS
# Stream depth flags give the agent enough buffer for PX4 v1.17's large topic set
# and allow it to handle reconnections without exhausting entity slots.
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce_agent \
    && cd /tmp/xrce_agent \
    && cmake -B build \
        -DUXRCE_BUILD_EXAMPLES=OFF \
        -DUAGENT_CONFIG_RELIABLE_STREAM_DEPTH=64 \
        -DUAGENT_CONFIG_BEST_EFFORT_STREAM_DEPTH=64 \
        -DUAGENT_CONFIG_MAX_TOPIC_SIZE=65536 \
    && cmake --build build -j$(nproc) \
    && cmake --install build \
    && rm -rf /tmp/xrce_agent

# px4_msgs: required by any node that subscribes to PX4 topics
RUN . /opt/ros/jazzy/setup.sh \
    && mkdir -p /opt/px4_msgs_ws/src \
    && git clone --depth 1 --branch release/1.17 \
       https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs \
    && colcon build \
       --base-paths /opt/px4_msgs_ws/src \
       --build-base  /opt/px4_msgs_ws/build \
       --install-base /opt/px4_msgs_ws/install \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/px4_msgs_ws/build

# Source ROS 2 and px4_msgs in every interactive shell
RUN echo "source /opt/ros/jazzy/setup.bash"            >> /root/.bashrc \
 && echo "source /opt/px4_msgs_ws/install/setup.bash"  >> /root/.bashrc

# PX4 artifacts copied from the builder stage; the host provides nothing
COPY --from=builder /px4_rt/ /px4/
RUN chmod +x /px4/build/px4_sitl_default/bin/px4

# QGroundControl
# AppImage is extracted rather than run directly to avoid the FUSE dependency.
# --appimage-extract unpacks the squashfs into squashfs-root without mounting anything.
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget \
        libpulse0 \
        libxcb-xinerama0 \
        libxkbcommon-x11-0 \
        libxcb-icccm4 \
        libxcb-image0 \
        libxcb-keysyms1 \
        libxcb-randr0 \
        libxcb-render-util0 \
        libxcb-xfixes0 \
        libnss3 \
        libgstreamer1.0-0 \
        libgstreamer-gl1.0-0 \
        libgstreamer-plugins-base1.0-0 \
        gstreamer1.0-plugins-good \
        gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

RUN wget -q https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.4/QGroundControl.AppImage \
        -O /tmp/QGroundControl.AppImage \
    && chmod +x /tmp/QGroundControl.AppImage \
    && cd /tmp && ./QGroundControl.AppImage --appimage-extract \
    && mv /tmp/squashfs-root /opt/QGroundControl \
    && rm /tmp/QGroundControl.AppImage \
    && chmod -R a+rX /opt/QGroundControl \
    && ln -s /opt/QGroundControl/AppRun /usr/local/bin/QGroundControl

# Dedicated non-root user for QGroundControl.
# QGC refuses to run cleanly as root and prints a blocking warning.
# The qgc service in docker-compose runs as this user via the user: directive.
RUN useradd -m -s /bin/bash qgcuser \
    && mkdir -p /home/qgcuser/.config/QGroundControl.org \
    && chown -R qgcuser:qgcuser /home/qgcuser

# Project mount point
WORKDIR /alerion_sim

ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash \
  && source /opt/px4_msgs_ws/install/setup.bash \
  && ros2 launch /alerion_sim/launch/simulation.launch.py \"$@\"", "--"]

CMD ["level:=full"]
