FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo11 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-serial \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

COPY environment/requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace
COPY . /workspace
RUN python3 -m pytest -q

CMD ["bash"]
