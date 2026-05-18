FROM osrf/ros:jazzy-desktop

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Dependencias del sistema
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-opencv \
    python3-pyqt5 \
    ros-jazzy-cv-bridge \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Dependencias Python (djitellopy no está en apt)
RUN pip install djitellopy --ignore-installed numpy && \
    pip install "numpy<2" --force-reinstall

# Copiar el workspace y el entrypoint
WORKDIR /ros2_ws
COPY src/ src/
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Compilar el paquete ROS 2
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select tello_control"

# Source automático en cualquier terminal nueva (docker exec incluido)
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "tello_control", "tello_launch.py"]
