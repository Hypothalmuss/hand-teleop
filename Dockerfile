# Webcam hand teleop: ROS 2 Humble + MuJoCo + MediaPipe, workspace built in the image.
# Build:  docker/run.sh build      Run: docker/run.sh teleop      (see README, "Docker")
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions libeigen3-dev ffmpeg xterm v4l-utils \
      ros-humble-launch-testing-ament-cmake ros-humble-launch-testing-ros \
      ros-humble-ament-cmake-gtest ros-humble-diagnostic-msgs ros-humble-visualization-msgs \
      ros-humble-tf2-ros \
      # OpenGL / EGL (NVIDIA driver libs are injected at run time; Mesa is the fallback)
      libglvnd0 libgl1 libegl1 libgles2 libgl1-mesa-dri libegl-mesa0 \
      # X11 / GUI libraries for the MuJoCo viewer (GLFW) and OpenCV windows (Qt xcb)
      libglib2.0-0 libsm6 libxext6 libxrender1 libxkbcommon-x11-0 libxcb-icccm4 libxcb-image0 \
      libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 libxcb-xinerama0 \
      libxcb-xkb1 libxcursor1 libxinerama1 libxrandr2 libxi6 libfontconfig1 libdbus-1-3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

COPY src src
COPY config config
COPY scripts scripts
COPY pyproject.toml LICENSE THIRD_PARTY_LICENSES.md ./
RUN . /opt/ros/humble/setup.sh && colcon build --event-handlers console_cohesion- \
    && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh && chmod -R a+rX /ws

# config/ is normally mounted from the host so calibration persists across runs.
ENV HAND_TELEOP_CONFIG=/ws/config \
    ROS_DOMAIN_ID=77 \
    ROS_LOCALHOST_ONLY=1 \
    ROS_LOG_DIR=/tmp/ros_log \
    HOME=/tmp/home \
    NVIDIA_DRIVER_CAPABILITIES=all

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "hand_teleop_bringup", "teleop.launch.py", "console:=xterm"]
