FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Repositorio OSRF para Gazebo Harmonic
RUN apt-get update && apt-get install -y curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
       http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# Dependencias del sistema
#   ros-jazzy-ros-gz-*        Puentes Gazebo ↔ ROS 2
#   gz-harmonic               Gazebo Harmonic (incluye python3-gz-transport13,
#                             python3-gz-msgs10 como dependencias transitivas)
#   python3-gz-transport13    Bindings Python de gz-transport  (wind_turbulence,
#   python3-gz-msgs10           suscripción RTF de validation_node)
#   ros-jazzy-px4-msgs        Tipos de mensajes px4_msgs
#   python3-psutil            CPU/RAM por proceso            (validation_node)
#   sysstat / htop / procps   pidstat, monitores de procesos
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
    && rm -rf /var/lib/apt/lists/*

# Activa la recolección de datos de sysstat (pidstat lo necesita en algunos modos)
RUN sed -i 's/ENABLED="false"/ENABLED="true"/' /etc/default/sysstat || true

# Herramientas Python extra 
RUN pip3 install --no-cache-dir --break-system-packages \
    jinja2 \
    pyyaml \
    psutil

# Agente MicroXRCE-DDS  (puente PX4 ↔ ROS 2 sobre RTPS/DDS)
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce_agent \
    && cd /tmp/xrce_agent \
    && cmake -B build -DUXRCE_BUILD_EXAMPLES=OFF \
    && cmake --build build -j$(nproc) \
    && cmake --install build \
    && rm -rf /tmp/xrce_agent


# px4_msgs  (requerido por cualquier nodo que se suscriba a topics de PX4)
# Fijado a release/1.15 
RUN . /opt/ros/jazzy/setup.sh \
    && mkdir -p /opt/px4_msgs_ws/src \
    && git clone --depth 1 --branch release/1.15 \
       https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs \
    && colcon build \
       --base-paths /opt/px4_msgs_ws/src \
       --build-base  /opt/px4_msgs_ws/build \
       --install-base /opt/px4_msgs_ws/install \
       --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /opt/px4_msgs_ws/build

# Carga ROS 2 y px4_msgs en cada shell interactivo
RUN echo "source /opt/ros/jazzy/setup.bash"            >> /root/.bashrc \
 && echo "source /opt/px4_msgs_ws/install/setup.bash"  >> /root/.bashrc

# Punto de montaje del proyecto
WORKDIR /alerion_sim

# Entrada por defecto: lanza la simulación; level=full salvo que se sobreescriba con CMD.
# Los scripts de Python se llaman por ruta directa, no requieren colcon build.
ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash \
  && source /opt/px4_msgs_ws/install/setup.bash \
  && ros2 launch /alerion_sim/launch/simulation.launch.py \"$@\"", "--"]

CMD ["level:=full"]
