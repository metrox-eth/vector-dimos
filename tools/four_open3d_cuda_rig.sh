#!/bin/bash
# Twin open3d-CUDA build oven, on the RIG under qemu-aarch64 - compiling this
# on a Jetson is slow while a much bigger idle machine is available. Runs INSIDE
# the official nvcr.io/nvidia/l4t-jetpack:r36.4.0 container (CUDA 12.6 aarch64
# included). The rover compiles natively in parallel: the first oven to produce
# a cp312 wheel wins. Started by docker run, /work = ~/open3d_four_rig mounted.
set -uo pipefail
log() { echo "[$(date +%H:%M:%S)] $*"; }
export DEBIAN_FRONTEND=noninteractive
cd /work

log "ETAPE 1/4: python 3.12 (deadsnakes) + outils"
apt-get update -qq > apt.log 2>&1
apt-get install -y -qq software-properties-common git > apt.log 2>&1 \
  || { log "ECHEC apt outils (apt.log)"; exit 1; }
add-apt-repository -y ppa:deadsnakes/ppa >> apt.log 2>&1
apt-get update -qq >> apt.log 2>&1
apt-get install -y -qq python3.12-dev python3.12-venv python3-pip >> apt.log 2>&1 \
  || { log "ECHEC apt python3.12 (apt.log)"; exit 1; }
log "python: $(python3.12 --version)"

log "ETAPE 2/4: dependances Open3D (leur script officiel)"
cd /work/Open3D
SUDO=" " bash util/install_deps_ubuntu.sh assume-yes > ../deps.log 2>&1 \
  || { log "ECHEC deps (deps.log)"; exit 1; }
# apt's python3-pip is the system 3.10 pip; the deadsnakes 3.12 needs its own
# (failure observed: cmake not found, because it was never installed for 3.12)
python3.12 -m ensurepip --upgrade >> ../deps.log 2>&1
python3.12 -m pip install -q "cmake>=3.24,<4" ninja setuptools wheel >> ../deps.log 2>&1 \
  || { log "ECHEC pip cmake (deps.log)"; exit 1; }
export PATH=/usr/local/cuda/bin:$PATH
log "nvcc: $(nvcc --version | tail -1)"
log "cmake: $(cmake --version | head -1)"

log "ETAPE 3/4: configuration (CUDA sm_87 Orin, sans GUI/WebRTC/tests)"
rm -rf build && mkdir -p build && cd build
cmake -DBUILD_CUDA_MODULE=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_GUI=OFF -DBUILD_WEBRTC=OFF \
      -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_MODULE=ON \
      -DPython3_EXECUTABLE=$(which python3.12) -DCMAKE_BUILD_TYPE=Release .. > ../../cmake.log 2>&1 \
  || { log "ECHEC cmake (cmake.log)"; exit 1; }

log "ETAPE 4/4: make -j6 pip-package (qemu: lent, plafonne pour ne pas starve other workloads)"
make -j6 pip-package > ../../make.log 2>&1 \
  || { log "ECHEC make (make.log, dernieres lignes:)"; tail -5 ../../make.log; exit 1; }

log "ROUE PRETE:"
ls -la lib/python_package/pip_package/*.whl
cp lib/python_package/pip_package/*.whl /work/ 2>/dev/null
