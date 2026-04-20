#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /firebot_ws/install/setup.bash

exec "$@"
