# Guía de instalación — alerion_sim

Entorno: Ubuntu 24.04 con GPU dedicada recomendado.


1. Requisitos previos del sistema
2. Clonar los repositorios
4. Construir la imagen Docker
5. Ejecutar la simulación
6. Instalación en host (sin Docker) 
7. Variables de entorno de referencia
8. Solución de problemas frecuentes


## 1. Requisitos previos del sistema

### Paquetes del sistema

sudo apt update && sudo apt install -y \
    git curl wget build-essential cmake ninja-build \
    python3-pip python3-venv python3-jinja2 python3-jsonschema \
    python3-numpy python3-packaging python3-toml \
    xorg openbox x11-xserver-utils

### Docker Engine

curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
newgrp docker

# Comprobar
docker --version
docker compose version

### Drivers de GPU (para renderizado Gazebo)

# NVIDIA — instalar el driver recomendado
sudo ubuntu-drivers install

# Verificar
nvidia-smi

# Si el equipo usa GPU integrada Intel/AMD el renderizado funciona igual;
# solo afecta al rendimiento de Gazebo en el nivel full.



## 2. Clonar los repositorios

Se necesitan dos repositorios en el mismo equipo:

# 1. Firmware PX4 (rama principal; se compilará para SITL)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/Desktop/PX4-Autopilot
cd ~/Desktop/PX4-Autopilot

# 2. Este proyecto
git clone <https://github.com/garaietxebe/alerion_sim> ~/Desktop/alerion_sim

# por defecto el docker-compose busca PX4 en /Desktop/PX4-Autopilot.
# Si se cambia de entorno, hay que sobreescribir la ruta cada sesion
 export PX4_DIR=/ruta/a/PX4-Autopilot


### Dependencias Python de PX4

# creamos las dependencias en un entorno virtual para que no salte el error de externally-managed-environments

cd ~/Desktop/PX4-Autopilot
python3 -m venv ~/.venv/px4
source ~/.venv/px4/bin/activate
pip3 install -r Tools/setup/requirements.txt
deactivate



## 3. Compilar PX4 para SITL

El binario compilado se monta en el contenedor Docker como volumen de solo lectura —
no se copia dentro de la imagen. Hay que compilarlo una sola vez en el host.

cd ~/Desktop/PX4-Autopilot

# Instalar dependencias del toolchain (script oficial)
bash Tools/setup/ubuntu.sh --no-nuttx

# Compilar el target SITL (tarda ~5-15 min la primera vez)
make px4_sitl_default

# Comprobar que el binario existe
ls build/px4_sitl_default/bin/px4

# Si la compilación falla por dependencias que faltan:

sudo apt install -y gcc-arm-none-eabi libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad libgstrtspserver-1.0-dev



## 4. Construir la imagen Docker

cd ~/Desktop/alerion_sim/alerion_sim

docker compose -f docker/docker-compose.yml build

# La imagen se llama `alerion_sim:latest` e incluye:

# ROS 2 | Jazzy 
# Gazebo | Harmonic 
# px4_msgs | release/1.15 
# Micro-XRCE-DDS Agent | latest main 
# Python bindings gz | gz-transport13 + gz-msgs10 



## 5. Ejecutar la simulación

### Preparar la pantalla (una sola vez por sesión de terminal)

# Permite al contenedor acceder al servidor X del host
xhost +local:docker

### Lanzar

cd ~/Desktop/alerion_sim/alerion_sim

# Nivel completo (alta fidelidad, por defecto)
docker compose -f docker/docker-compose.yml run --rm sim level:=full

# Nivel development (sin ruido, sin distorsión de cámara)
docker compose -f docker/docker-compose.yml run --rm sim level:=development

# Nivel development con perfil de visión
docker compose -f docker/docker-compose.yml run --rm sim \
    level:=development sensor_profile:=vision

# Shell interactivo con el mismo entorno
docker compose -f docker/docker-compose.yml run --rm shell

### Validación / monitorización de recursos (terminal separado)

# Mientras la simulación está en marcha:
docker compose -f docker/docker-compose.yml run --rm validate
# Genera: /tmp/alerion_validation.csv  y  /tmp/alerion_compute.csv

### Ver topics ROS 2 desde el host

El contenedor usa `network_mode: host`, así que los topics son visibles
directamente si ROS 2 está instalado en el host:


source /opt/ros/jazzy/setup.bash
ros2 topic list




## 6. Instalación sin Docker


### 6.1 ROS 2 Jazzy

# Repositorio oficial
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update && sudo apt install -y ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

### 6.2 Gazebo Harmonic + puentes ROS 2

sudo apt install -y \
    gz-harmonic \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-image \
    ros-jazzy-nav-msgs \
    ros-jazzy-sensor-msgs \
    python3-gz-transport13 \
    python3-gz-msgs10

### 6.3 px4_msgs

mkdir -p ~/px4_msgs_ws/src
git clone --depth 1 --branch release/1.15 \
    https://github.com/PX4/px4_msgs.git ~/px4_msgs_ws/src/px4_msgs

source /opt/ros/jazzy/setup.bash
colcon build --base-paths ~/px4_msgs_ws/src \
             --build-base  ~/px4_msgs_ws/build \
             --install-base ~/px4_msgs_ws/install \
             --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "source ~/px4_msgs_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

### 6.4 Micro-XRCE-DDS Agent

git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent
cmake -B build -DUXRCE_BUILD_EXAMPLES=OFF
cmake --build build -j$(nproc)
sudo cmake --install build
sudo ldconfig /usr/local/lib/

### 6.5 Dependencias Python adicionales

pip3 install --user jinja2 pyyaml psutil numpy

### 6.6 Lanzar sin Docker

export PX4_DIR=~/Desktop/PX4-Autopilot

source /opt/ros/jazzy/setup.bash
source ~/px4_msgs_ws/install/setup.bash

ros2 launch ~/Desktop/alerion/alerion_sim/launch/simulation.launch.py level:=full





## Solución de problemas frecuentes

### Gazebo no muestra ventana / error de display


# Antes de lanzar el contenedor:
xhost +local:docker
export DISPLAY=:0

### Errores de timestamp IMU / dron oscila

Síntoma: `ERROR [vehicle_imu] timestamp error timestamp_sample: X, previous: Y`

Causa: la física a 500 Hz (`max_step_size: 0.002`) satura la CPU

physics:
  max_step_size: 0.004   # 250 Hz — valor estable (igual que el nivel development)

### QoS mismatch — topics de cámara vacíos

`ros_gz_image` publica con `BEST_EFFORT`. Los suscriptores deben usar el mismo
perfil o DDS descarta los mensajes. 

### `camera_distorted` no publica o publica una imagen grayscale estatica

`Camera_distorted` todavia no funciona.

### Errores de mallas del modelo (`model://x500_base/meshes/...`)

Gazebo necesita encontrar los modelos de PX4 en `GZ_SIM_RESOURCE_PATH`.
El launch file exporta esta variable automáticamente. Si el error persiste,
verificar que `PX4_DIR` apunta al repositorio correcto y que está compilado.

