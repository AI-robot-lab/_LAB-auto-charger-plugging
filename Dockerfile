# Używamy oficjalnego obrazu ROS 2 Humble w wersji bazowej
FROM ros:humble-ros-base

# Ustawienie zmiennych środowiskowych
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# Instalacja podstawowych narzędzi systemowych
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Tworzenie folderu roboczego (workspace)
WORKDIR /ros2_ws

# Kopiowanie kodu źródłowego do obrazu
COPY ./src ./src

# Instalacja zależności przy użyciu rosdep
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# Kompilacja projektu
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Automatyczne ładowanie środowiska ROS po wejściu do kontenera
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Komenda startowa
CMD ["bash"]
