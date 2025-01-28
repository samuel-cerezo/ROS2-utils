# Usar la imagen base de ROS2 Humble Desktop
FROM osrf/ros:humble-desktop

# Actualizar e instalar dependencias necesarias
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    build-essential \
    software-properties-common \
    libssl-dev \
    libusb-1.0-0-dev \
    libacl1-dev \
    python3-pip \
    gnome-terminal \
    dbus-x11 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# Instalar los paquetes ROS2 necesarios para rosbag2
RUN apt-get update && apt-get install -y \
    ros-humble-rosbag2* \
    ros-humble-ros2-control \
    ros-humble-xacro \
    ros-humble-forward-command-controller \
    ros-humble-generate-parameter-library \
    ros-humble-rviz2 \
    ros-humble-position-controllers \
    ros-humble-joint-trajectory-controller \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Clonar librealsense e instalar
RUN git clone https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release && \
    make -j$(nproc) && \
    make install && \
    cd ../..

# Instalar reglas UDEV para RealSense
COPY files/99-realsense.rules /etc/udev/rules.d/99-realsense.rules

# Copiar el workspace (ros2_ws) desde la carpeta 'files' a la imagen
COPY files/ros2_ws /home/ros2_ws

# Establecer el directorio de trabajo
WORKDIR /home/ros2_ws

# Preparar e instalar dependencias del workspace
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Instalar librerias
RUN pip install --target=/opt/ros/humble/lib/python3.10/site-packages rosbags
RUN pip install rosbags-image
RUN pip install numpy==1.24.4
RUN pip install pandas

RUN apt-get update

RUN pip install pyrealsense2
RUN pip install spatialmath-python

RUN pip install git+https://github.com/petercorke/robotics-toolbox-python.git
RUN pip install roboticstoolbox-python

# Construir el workspace con symlink-install
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Modificar el .bashrc para configurar el entorno automáticamente
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home

# Establecer el comando predeterminado para iniciar bash
CMD ["bash"]
