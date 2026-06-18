# Stage 1 — builder
# Misma imagen base que el runtime para garantizar ABI idéntico en los plugins de Gazebo.
# Los .so compilados aquí se cargan en el mismo gz-harmonic → sin mismatch de símbolos.

FROM osrf/ros:jazzy-desktop AS builder

ARG PX4_VERSION=v1.17.0
ARG DEBIAN_FRONTEND=noninteractive

# Repositorio OSRF — necesario para gz-harmonic (compilación de plugins)
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

# Clonar PX4 con todos sus submódulos (shallow para reducir tráfico de red)
RUN git clone --recursive --depth 1 --branch ${PX4_VERSION} \
    https://github.com/PX4/PX4-Autopilot.git /px4_src

# ubuntu.sh instala todas las dependencias de compilación y Python correctas
# para la versión clonada (gcc, cmake, gz, gstreamer, jinja2, etc.)
# RUNS_IN_DOCKER=true  → el script instala sudo antes de usarlo
# USER=root            → evita que "usermod -aG dialout $USER" falle con variable vacía
# --no-nuttx           → omite la toolchain de hardware (Pixhawk), solo necesitamos SITL
RUN cd /px4_src \
    && RUNS_IN_DOCKER=true USER=root DEBIAN_FRONTEND=noninteractive \
       bash Tools/setup/ubuntu.sh --no-nuttx

# Compilar target SITL (~15 min en un servidor con 8 cores)
RUN cd /px4_src && make -j$(nproc) px4_sitl_default

# Preparar directorio de staging con solo los artefactos de runtime.
# cp -rL resuelve symlinks en origen (p. ej. rootfs/etc -> ../etc)
# de forma que el COPY del stage final no encuentra symlinks rotos.
RUN set -e; \
    B=/px4_src/build/px4_sitl_default; \
    T=/px4_rt; \
    \
    # binario + scripts de arranque (etc/) + rootfs
    mkdir -p $T/build/px4_sitl_default; \
    cp -rL $B/bin    $T/build/px4_sitl_default/; \
    cp -rL $B/etc    $T/build/px4_sitl_default/; \
    cp -rL $B/rootfs $T/build/px4_sitl_default/; \
    \
    # plugins de Gazebo compilados (.so) — cargados por GZ_SIM_SYSTEM_PLUGIN_PATH
    mkdir -p $T/build/px4_sitl_default/src/modules/simulation; \
    cp -rL $B/src/modules/simulation/gz_plugins \
           $T/build/px4_sitl_default/src/modules/simulation/; \
    \
    # modelos y mundos de Gazebo — cargados por GZ_SIM_RESOURCE_PATH
    mkdir -p $T/Tools/simulation/gz; \
    cp -rL /px4_src/Tools/simulation/gz/models $T/Tools/simulation/gz/; \
    cp -rL /px4_src/Tools/simulation/gz/worlds $T/Tools/simulation/gz/; \
    \
    # modelos de Gazebo-classic (mallas DAE: estantes, cajas, palés)
    mkdir -p $T/Tools/simulation/gazebo-classic/sitl_gazebo-classic; \
    cp -rL /px4_src/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models \
           $T/Tools/simulation/gazebo-classic/sitl_gazebo-classic/; \
    \
    # server.config — lista de plugins de sensores que carga PX4 en Gazebo
    mkdir -p $T/src/modules/simulation/gz_bridge; \
    cp /px4_src/src/modules/simulation/gz_bridge/server.config \
       $T/src/modules/simulation/gz_bridge/


# Stage 2 — runtime
# Imagen final: ROS 2 Jazzy + Gazebo Harmonic + artefactos PX4 del builder.

FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# PX4_DIR apunta al árbol copiado desde el builder; el launch file lo lee
ENV PX4_DIR=/px4

# Repositorio OSRF — necesario para gz-harmonic y sus bindings Python
# (osrf/ros:jazzy-desktop solo incluye el repo de ROS, no el de Gazebo)
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

# Dependencias del sistema
#   ros-jazzy-ros-gz-*        Puentes Gazebo ↔ ROS 2
#   gz-harmonic               Gazebo Harmonic (incluye python3-gz-transport13,
#                             python3-gz-msgs10 como dependencias transitivas)
#   python3-gz-transport13    Bindings Python de gz-transport  (wind_turbulence,
#   python3-gz-msgs10           suscripción RTF de validation_node)
#   python3-psutil            CPU/RAM por proceso              (validation_node)
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

# Activa la recolección de datos de sysstat
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
    && git clone --depth 1 --branch release/1.17 \
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

# Artefactos PX4 copiados desde el builder — no se necesita nada del host
COPY --from=builder /px4_rt/ /px4/
RUN chmod +x /px4/build/px4_sitl_default/bin/px4

# Punto de montaje del proyecto
WORKDIR /alerion_sim

ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/jazzy/setup.bash \
  && source /opt/px4_msgs_ws/install/setup.bash \
  && ros2 launch /alerion_sim/launch/simulation.launch.py \"$@\"", "--"]

CMD ["level:=full"]
