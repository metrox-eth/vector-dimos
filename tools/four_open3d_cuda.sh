#!/bin/bash
# open3d-CUDA build oven ("four"), native on the Orin - first run 2026-08-27
# evening. Why: the dimOS mapper asks for CUDA by default, our packages are
# CPU-only, and no prebuilt open3d-CUDA Jetson wheel exists to download
# (checked). Runs detached, writes everything to ~/open3d_four/four.log.
# Prerequisites, to put in place before launching: Open3D v0.19.0 cloned into
# ~/open3d_four/Open3D, an 8 GB swapfile active, the VECTOR stack stopped.
set -uo pipefail
log() { echo "[$(date +%H:%M:%S)] $*"; }
cd ~/open3d_four

log "ETAPE 1/4: cuda-toolkit-12-6 + python3-pip (apt, ~3 Go, depot NVIDIA du JetPack)"
sudo -n env DEBIAN_FRONTEND=noninteractive apt-get install -y cuda-toolkit-12-6 python3-pip > apt_cuda.log 2>&1 \
  || { log "ECHEC apt cuda (voir apt_cuda.log)"; exit 1; }
export PATH=/usr/local/cuda-12.6/bin:$PATH
log "nvcc: $(nvcc --version | tail -1)"

log "ETAPE 2/4: dependances Open3D (leur script officiel)"
cd Open3D
sudo -n env DEBIAN_FRONTEND=noninteractive bash util/install_deps_ubuntu.sh assume-yes > ../deps.log 2>&1 \
  || { log "ECHEC deps (voir deps.log)"; exit 1; }
python3 -m pip install --user -q "cmake>=3.24,<4" ninja setuptools wheel >> ../deps.log 2>&1
export PATH=$HOME/.local/bin:$PATH
log "cmake: $(cmake --version | head -1)"

log "ETAPE 3/4: configuration (CUDA sm_87 Orin, sans GUI/WebRTC/tests)"
rm -rf build && mkdir -p build && cd build
cmake -DBUILD_CUDA_MODULE=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_GUI=OFF -DBUILD_WEBRTC=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_MODULE=ON \
      -DPython3_EXECUTABLE="$HOME/vector-dimos/.venv/bin/python" -DCMAKE_BUILD_TYPE=Release .. > ../../cmake.log 2>&1 \
  || { log "ECHEC cmake (voir cmake.log)"; exit 1; }

log "ETAPE 4/4: make -j4 pip-package - le long morceau (des heures)"
make -j4 pip-package > ../../make.log 2>&1 \
  || { log "ECHEC make (voir make.log, dernieres lignes:)"; tail -5 ../../make.log; exit 1; }

log "ROUE PRETE:"
ls -la lib/python_package/pip_package/*.whl
