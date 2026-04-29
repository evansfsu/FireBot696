FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# Base deps for all platforms. python3-picamera2 exists only for arm64/armhf in Ubuntu
# (Pi camera stack); docker build on amd64 fails if we apt-install it here — vision_node
# falls back to OpenCV VideoCapture when picamera2 is missing.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-dev \
    libcap-dev \
    libgl1 \
    libglib2.0-0 \
    libcamera0 \
    v4l-utils \
 && rm -rf /var/lib/apt/lists/*

# Use case … esac so a missing || in [ … ] … [ … ] can’t break the shell (Dockerfile gotcha).
RUN arch="$(dpkg --print-architecture)" && \
    apt-get update && \
    case "$arch" in \
      arm64|armhf) \
        apt-get install -y --no-install-recommends python3-picamera2 \
          || echo "[docker] WARN: python3-picamera2 failed; vision_node uses OpenCV fallback" ;; \
      *) \
        echo "[docker] skipping python3-picamera2 on $arch (amd64 has no deb)" ;; \
    esac && \
    rm -rf /var/lib/apt/lists/*

# Ultralytics depends on PyTorch. Default PyPI torch wheels are CUDA (many nvidia-* packages,
# multi‑GB). Pi 5 has no NVIDIA GPU; use CPU builds from PyTorch's index only.
RUN pip3 install --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu \
 && pip3 install --no-cache-dir \
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
