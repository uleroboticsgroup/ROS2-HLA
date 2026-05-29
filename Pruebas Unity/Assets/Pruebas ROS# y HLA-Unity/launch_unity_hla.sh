#!/bin/bash
# Script para lanzar Unity con las variables de entorno necesarias para Pitch RTI HLA

# Rutas detectadas
JAVA_LIB_PATH="/usr/lib/jvm/java-21-openjdk-amd64/lib/server"
PITCH_RTI_PATH="/home/vicen/prti1516e/lib/gcc73_64"

# Añadir al LD_LIBRARY_PATH (necesario para que libhla_plugin.so encuentre libjvm.so)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$JAVA_LIB_PATH:$PITCH_RTI_PATH

echo "Configuracion de HLA:"
echo "  JAVA: $JAVA_LIB_PATH"
echo "  RTI:  $PITCH_RTI_PATH"

# Editor de Unity detectado
UNITY_EDITOR_PATH="$HOME/Unity/Hub/Editor/6000.3.4f1/Editor/Unity"

# Calcular la raíz del proyecto (asumiendo que este script está en Assets/Pruebas ROS-Unity)
# $BASH_SOURCE es este script.
# dirname -> Assets/Pruebas ROS-Unity
# dirname -> Assets
# dirname -> Pruebas Unity (Raíz del proyecto)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_PATH="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "Lanzando Editor: $UNITY_EDITOR_PATH"
echo "Proyecto:       $PROJECT_PATH"

if [ -f "$UNITY_EDITOR_PATH" ]; then
    "$UNITY_EDITOR_PATH" -projectPath "$PROJECT_PATH" &
else
    echo "ERROR: No se encontró el ejecutable de Unity en: $UNITY_EDITOR_PATH"
    echo "Por favor, verifica la ruta o abre Unity Hub manualmente tras ejecutar:"
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
fi
