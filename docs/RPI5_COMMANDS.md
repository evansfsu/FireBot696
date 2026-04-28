# Raspberry Pi 5 — command list

Assume 64-bit Raspberry Pi OS, repo cloned to `~/FireBot696`. Replace the path if yours differs.

---

## 1. Update from GitHub

```bash
cd ~/FireBot696
git status
git pull origin main
```

---

## 2. Docker (full stack)

Install once (if needed):

```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
```

Log out and back in after `usermod`. Plug in CSI camera and Arduino USB before starting.

Build and run (from repo root):

```bash
cd ~/FireBot696
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d firebot
```

Logs:

```bash
docker compose -f docker/docker-compose.yml logs -f firebot
```

Stop:

```bash
docker compose -f docker/docker-compose.yml down
```

Operator CLI (inside container):

```bash
cd ~/FireBot696
docker compose -f docker/docker-compose.yml exec firebot firebot alarm
docker compose -f docker/docker-compose.yml exec firebot firebot confirm
docker compose -f docker/docker-compose.yml exec firebot firebot confirm --deny
docker compose -f docker/docker-compose.yml exec firebot firebot estop
docker compose -f docker/docker-compose.yml exec firebot firebot status
```

Shell inside container:

```bash
docker compose -f docker/docker-compose.yml exec firebot bash
source /opt/ros/humble/setup.bash
source /firebot_ws/install/setup.bash
ros2 topic list
```

---

## 3. Dev profile (rebuild workspace from mounted source)

```bash
cd ~/FireBot696
docker compose -f docker/docker-compose.yml --profile dev up --build firebot-dev
```

Uses bind-mount on `firebot_ws/src`; slower startup because of `colcon build`.

---

## 4. Host Python — test scripts (outside Docker)

Install once on the Pi:

```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-picamera2 python3-serial
pip3 install --break-system-packages ultralytics
```

Repo root:

```bash
cd ~/FireBot696
```

Camera:

```bash
python3 scripts/rpi_test_camera.py --preview
python3 scripts/rpi_test_camera.py --width 640 --height 480 --frames 20
```

YOLO + camera (weights in `models/best_small.pt` by default):

```bash
python3 scripts/rpi_test_yolo_fire.py --video-mode
python3 scripts/rpi_test_yolo_fire.py --headless
python3 scripts/rpi_test_yolo_fire.py --model ~/FireBot696/models/best_small.pt --conf 0.2 --infer-every-n 3
```

Arduino USB serial (robot on blocks; short spin then estop):

```bash
python3 scripts/rpi_test_arduino_serial.py --port /dev/ttyACM0
python3 scripts/rpi_test_arduino_serial.py --dry-run
```

YOLO + ROS keys (desktop or VNC; stack must be running, workspace sourced):

```bash
cd ~/FireBot696/firebot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd ~/FireBot696
python3 scripts/rpi_test_yolo_fire.py --ros --video-mode
```

Keys with focus on the OpenCV window: `a` alarm on, `z` alarm off, `c` confirm, `x` deny, `e` estop, `h` help, `q` quit.

---

## 5. Serial port

```bash
ls -l /dev/ttyACM*
groups
```

If permission denied on `/dev/ttyACM0`:

```bash
sudo usermod -aG dialout $USER
```

Reboot or re-login after that.

---

## 6. Camera devices

```bash
ls -l /dev/video0 /dev/media* 2>/dev/null
libcamera-hello --list-cameras
```

---

## 7. Config file (edit before rebuild if you change defaults)

`firebot_ws/src/firebot/config/firebot_params.yaml`

After editing, rebuild the `firebot` image (or use `firebot-dev` so the file is picked up from the mount).

---

## Other docs

- Wiring: [WIRING.md](WIRING.md)
- Pi ↔ Mega: [INTEGRATION.md](INTEGRATION.md)
- Serial protocol: [PROTOCOL.md](PROTOCOL.md)
- Arduino Serial Monitor: [ARDUINO_TESTING.md](ARDUINO_TESTING.md)
