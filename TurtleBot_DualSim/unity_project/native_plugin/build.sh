#!/bin/bash
# Build script for the Unity HLA native plugin
# Produces libhla_plugin.so for Unity's Plugins/x86_64/ directory

set -e

PRTI_DIR="/home/vicen/prti1516e"
API_DIR="${PRTI_DIR}/api/cpp/HLA_1516-2010"
LIB_DIR="${PRTI_DIR}/lib/gcc73_64"
SRC="hla_plugin.cpp"
OUT_DIR="../Assets/Plugins/x86_64"

echo "=== Compilando plugin HLA nativo para Unity ==="

g++ -std=c++17 -shared -fPIC -O2 \
    -I"${API_DIR}" \
    -L"${LIB_DIR}" \
    -o "${OUT_DIR}/libhla_plugin.so" \
    "${SRC}" \
    -lrti1516e64 -lfedtime1516e64 \
    -Wl,-rpath,"${LIB_DIR}"

echo "=== Plugin compilado: ${OUT_DIR}/libhla_plugin.so ==="
echo "=== También copiando dependencias RTI ==="

# Copy RTI libs so Unity can find them
cp -v "${LIB_DIR}/librti1516e64.so" "${OUT_DIR}/"
cp -v "${LIB_DIR}/libfedtime1516e64.so" "${OUT_DIR}/"

echo "=== LISTO ==="
ls -la "${OUT_DIR}/"
