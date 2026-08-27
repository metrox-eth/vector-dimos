#!/bin/bash
# Four open3d-CUDA jumeau, sur le RIG sous qemu-aarch64 (idee metrox 27/08:
# "on ne va pas compiler un truc sur un Jetson, le rig dort") - tourne DANS
# le conteneur officiel nvcr.io/nvidia/l4t-jetpack:r36.4.0 (CUDA 12.6 aarch64
# inclus). Le rover compile en natif en parallele: le premier four qui sort
# une roue cp312 gagne. Lance par docker run, /work = ~/open3d_four_rig monte.
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
# python3-pip d'apt = le pip du 3.10 systeme; le 3.12 deadsnakes doit
# recevoir le sien (echec 18h20: cmake introuvable car jamais installe)
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

log "ETAPE 4/4: make -j6 pip-package (qemu: lent, plafonne pour ne pas gener Vita)"
make -j6 pip-package > ../../make.log 2>&1 \
  || { log "ECHEC make (make.log, dernieres lignes:)"; tail -5 ../../make.log; exit 1; }

log "ROUE PRETE:"
ls -la lib/python_package/pip_package/*.whl
cp lib/python_package/pip_package/*.whl /work/ 2>/dev/null
