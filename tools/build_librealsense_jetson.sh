#!/bin/bash
# Build librealsense + pyrealsense2 for the rover's venv (JetPack 6.2, glibc 2.35, py3.12).
# RSUSB backend = no kernel patch. Low priority so the drive loop keeps the CPU.
set -o pipefail
cd ~
echo "== $(date) apt deps"
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq git cmake build-essential libusb-1.0-0-dev libssl-dev pkg-config libudev-dev python3-dev || exit 1
if [ ! -d librealsense ]; then
  for TAG in v2.56.5 v2.56.4 v2.56.3 v2.55.1; do
    git clone -q --depth 1 --branch $TAG https://github.com/IntelRealSense/librealsense.git && { echo "cloned $TAG"; break; }
  done
fi
[ -d librealsense ] || { echo "clone failed"; exit 1; }
cd librealsense && mkdir -p build && cd build
echo "== $(date) cmake"
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON -DBUILD_PYTHON_BINDINGS=ON \
  -DPYTHON_EXECUTABLE=$HOME/vector-dimos/.venv/bin/python -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_WITH_CUDA=OFF -DBUILD_TOOLS=OFF -DBUILD_UNIT_TESTS=OFF || exit 1
echo "== $(date) make -j4 (nice 15)"
nice -n 15 make -j4 2>&1 | grep -E "^\[|error|Error" | tail -n 2000
echo "== $(date) install"
sudo make install > /dev/null || exit 1
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
SP=$HOME/vector-dimos/.venv/lib/python3.12/site-packages
ls $SP/pyrealsense2* 2>/dev/null || { mkdir -p $SP/pyrealsense2; cp wrappers/python/pyrealsense2*.so $SP/pyrealsense2/ && echo "from .pyrealsense2 import *" > $SP/pyrealsense2/__init__.py; }
sudo ldconfig
echo "== $(date) verify"
$HOME/vector-dimos/.venv/bin/python -c "import pyrealsense2 as rs; print('pyrealsense2 OK', rs.__version__); ctx=rs.context(); ds=ctx.query_devices(); print('devices', len(ds)); [print(d.get_info(rs.camera_info.name), d.get_info(rs.camera_info.serial_number), d.get_info(rs.camera_info.firmware_version)) for d in ds]"
echo "== $(date) DONE rc=$?"
