FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-dev \
    libcap-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libcamera0 \
    v4l-utils \
    python3-picamera2 \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    ultralytics \
    opencv-python-headless \
    pyserial

WORKDIR /firebot_ws

COPY firebot_ws/src /firebot_ws/src

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY models /models

COPY docker/entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "firebot", "firebot.launch.py"]
