# Używamy oficjalnego obrazu ROS 2 Humble w wersji bazowej
FROM ros:humble-ros-base

# Ustawienie zmiennych środowiskowych
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# Instalacja dodatkowych bibliotek dla GUI i narzędzi wizualnych
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    python3-rosdep \
    ros-humble-rviz2 \
    ros-humble-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

# Ustawienie zmiennej dla renderingu (pomaga przy braku dedykowanej karty w kontenerze)
ENV LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /ros2_ws
COPY ./src ./src

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
