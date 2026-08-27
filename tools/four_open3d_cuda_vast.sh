#!/bin/bash
# Four open3d-CUDA EXPRESS sur ARM loue (vast.ai GB10, 20 coeurs Grace natifs)
# - idee metrox 27/08 18h48 ("on a du credit sur vast.ai"). Tourne DANS le
# conteneur nvcr.io/nvidia/cuda:12.6-devel arm64 de l'instance: nvcc 12.6
# cible sm_87 (Orin) sans avoir besoin du GPU local. ~$0.33/h, detruite
# sitot la roue cp312 sortie. Les fours rover+rig restent en filet.
set -uo pipefail
log() { echo "[$(date +%H:%M:%S)] $*"; }
export DEBIAN_FRONTEND=noninteractive
cd /root

log "ETAPE 1/4: python 3.12 (deadsnakes) + outils"
apt-get update -qq > apt.log 2>&1
apt-get install -y -qq software-properties-common git wget > apt.log 2>&1 \
  || { log "ECHEC apt outils (apt.log)"; exit 1; }
add-apt-repository -y ppa:deadsnakes/ppa >> apt.log 2>&1
apt-get update -qq >> apt.log 2>&1
apt-get install -y -qq python3.12-dev python3.12-venv >> apt.log 2>&1 \
  || { log "ECHEC apt python3.12 (apt.log)"; exit 1; }
log "python: $(python3.12 --version)"

log "ETAPE 2/4: sources v0.19.0 + dependances Open3D"
[ -d Open3D ] || git clone --depth 1 --branch v0.19.0 https://github.com/isl-org/Open3D.git > clone.log 2>&1
cd Open3D
# patch Pair CUDA (bug Jetson connu, isl-org/Open3D#6885): le constructeur
# fourni rend le tableau __shared__ non-trivial, nvcc recent refuse (20054)
sed -i 's|constexpr __device__ inline Pair() {}|Pair() = default;|' cpp/open3d/core/nns/kernel/Pair.cuh
grep -q "Pair() = default" cpp/open3d/core/nns/kernel/Pair.cuh || { log "ECHEC patch Pair"; exit 1; }
SUDO=" " bash util/install_deps_ubuntu.sh assume-yes > ../deps.log 2>&1 \
  || { log "ECHEC deps (deps.log)"; exit 1; }
python3.12 -m ensurepip --upgrade >> ../deps.log 2>&1
python3.12 -m pip install -q "cmake>=3.24,<4" ninja setuptools wheel >> ../deps.log 2>&1 \
  || { log "ECHEC pip cmake (deps.log)"; exit 1; }
export PATH=/usr/local/cuda/bin:$PATH
log "nvcc: $(nvcc --version | tail -1)"
log "cmake: $(cmake --version | head -1)"

log "ETAPE 3/4: configuration (CUDA sm_87 Orin, sans GUI/WebRTC/tests)"
rm -rf build && mkdir -p build && cd build
# CUDA_RUNTIME_LIBRARY=Shared: le cudart STATIQUE injecte un segment TLS
# 48 Ko aligne 4096 dans le module python, et la glibc 2.35 du rover ne
# sait pas le placer au dlopen (mesure 27/08 19h40: tunables sans effet,
# mini-repro nvcc: -cudart shared = plus de segment TLS du tout).
cmake -DBUILD_CUDA_MODULE=ON -DCMAKE_CUDA_ARCHITECTURES=87 -DBUILD_GUI=OFF -DBUILD_WEBRTC=OFF \
      -DCMAKE_CUDA_RUNTIME_LIBRARY=Shared \
      -DBUILD_EXAMPLES=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_PYTHON_MODULE=ON \
      -DPython3_EXECUTABLE=$(which python3.12) -DCMAKE_BUILD_TYPE=Release .. > ../../cmake.log 2>&1 \
  || { log "ECHEC cmake (cmake.log)"; exit 1; }

log "ETAPE 4/4: make -j20 pip-package (20 coeurs Grace natifs)"
make -j20 pip-package > ../../make.log 2>&1 \
  || { log "ECHEC make (make.log, dernieres lignes:)"; tail -5 ../../make.log; exit 1; }

log "VERIFICATION avant livraison (meme glibc 2.35 que le rover):"
W=$(ls lib/python_package/pip_package/*.whl | head -1)
python3.12 -m pip install -q "$W" numpy > /root/verif.log 2>&1
SO=$(python3.12 -c "import open3d, os; print(os.path.dirname(open3d.__file__))" 2>/dev/null)/cuda/pybind.cpython-312-aarch64-linux-gnu.so
[ -f "$SO" ] || SO=$(find / -name "pybind.cpython-312*.so" -path "*open3d*" 2>/dev/null | head -1)
log "segment TLS: $(readelf -lW $SO | grep TLS | head -1)"
python3.12 -c "import ctypes; ctypes.CDLL(\"$SO\")" && log "CHARGEMENT OK - roue saine" || { log "ECHEC chargement - roue invalide"; exit 1; }
log "ROUE PRETE:"
ls -la lib/python_package/pip_package/*.whl
cp lib/python_package/pip_package/*.whl /root/ 2>/dev/null
