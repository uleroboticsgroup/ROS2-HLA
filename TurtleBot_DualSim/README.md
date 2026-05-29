# TurtleBot DualSim — Sincronización Gazebo ↔ Unity via HLA

Sistema de doble simulación donde un TurtleBot es simulado independientemente en **Gazebo (ROS2)** y en **Unity**. Mediante una federación HLA (IEEE 1516e) con sincronización temporal, cada simulador envía la pose de su robot y muestra un **ghost (dummy)** del robot remoto.

## Arquitectura

```
  GAZEBO (ROS2)                    HLA RTI                    UNITY
 ┌─────────────────┐    ┌─────────────────────────┐    ┌─────────────────┐
 │ TurtleBot propio │    │  Federation "DualSimFed" │    │ TurtleBot propio│
 │ Ghost Unity ◀───┼────┼── Obj: UnityRobot       │    │ Ghost Gazebo    │
 │  (marker RViz)  │    │  Obj: GazeboRobot ──────┼───▶│  (prefab ghost) │
 │  Bridge Node     │    │  Time: Ambos Hybrid      │    │ DualSimController│
 └─────────────────┘    └─────────────────────────┘    └─────────────────┘
```

## Estructura del proyecto

```
TurtleBot_DualSim/
├── ros2_ws/                     # Workspace ROS2
│   └── src/dual_sim_bridge/     # Paquete ROS2
│       ├── dual_sim_bridge/
│       │   ├── bridge_node.py   # Nodo principal del bridge
│       │   └── hla_manager.py   # Gestión HLA (float32LE!)
│       ├── config/
│       │   └── gazebo_bridge.yaml
│       ├── fom/
│       │   └── DualSimFOM.xml   # FOM compartido
│       ├── launch/
│       │   └── gazebo_bridge.launch.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── unity_project/               # Assets para Unity
    └── Assets/
        ├── FOM/DualSimFOM.xml
        └── Scripts/
            ├── HlaInterface.cs
            ├── DualSimNetworkManager.cs
            └── DualSimController.cs
```

## Prerrequisitos

- **ROS2 Jazzy** con `turtlebot4_gz_bringup` instalado
- **Pitch pRTI** en `/home/vicen/prti1516e/`
- **Unity 2022+** con el plugin HLA C++ (`hla_plugin.so`) en `Assets/Plugins/x86_64/`
- **JPype** instalado en Python (`pip install jpype1`)

## Cómo lanzar

### 1. Iniciar Pitch RTI
Arrancar el servidor RTI de Pitch antes de lanzar cualquier federado.

### 2. Lado Gazebo (ROS2)

```bash
cd /home/vicen/ISDEFE/TurtleBot_DualSim/ros2_ws
colcon build --packages-select dual_sim_bridge
source install/setup.bash

# Lanzar con el mapa Dual Rooms (compartido con Unity)
ros2 launch dual_sim_bridge gazebo_bridge.launch.py world:=dual_rooms

# Lanzar con mundo vacío (default)
ros2 launch dual_sim_bridge gazebo_bridge.launch.py

```

Esto lanza:
- Simulación TurtleBot4 en Gazebo (si turtlebot4_gz_bringup está disponible)
- Bridge HLA que publica la pose del robot como objeto `Box` en la federación `DualSimFed`
- RViz para visualizar el ghost del robot de Unity

### 3. Lado Unity

1. Abrir el proyecto **"Pruebas Unity"** (o un proyecto existente con el setup preparado).
2. Copiar los archivos de `unity_project/Assets/` al proyecto si es nuevo.
3. Copiar el plugin HLA C++ a `Assets/Plugins/x86_64/`:
   - `libhla_plugin.so`, `librti1516e64.so`, `libfedtime1516e64.so`
4. En la escena:
   - Crear un GameObject vacío y adjuntar `DualRoomsBuilder.cs`. Ajustar `Ground Y` al valor del suelo de la escena.
   - Si se usa `world:=dual_rooms` en Gazebo, al pulsar Play se generarán las paredes idénticas automáticamente.
5. Play → el robot de Unity se conecta a `DualSimFed` y empieza a intercambiar poses con Gazebo.

## Sincronización Temporal HLA

Ambos federados usan modo **Hybrid** (Time Regulating + Time Constrained):
- Cada lado avanza el tiempo lógico a su propio ritmo pero coordinado con el otro
- El `time_step` es 0.05s (20 Hz) en ambos lados
- Un pacer de tiempo real en el bridge ROS2 evita que el tiempo lógico se desincronice del reloj de pared
- Los `sync_points` (`ReadyToRun`) aseguran que ambos lados arrancan sincronizados

## Convenciones de coordenadas (Mapeo directo)

| Gazebo | Unity | HLA |
|--------|-------|-----|
| X (adelante) | X (derecha) | PositionX |
| Y (izquierda) | Z (adelante) | PositionY |
| Z (arriba) | Y (arriba) | PositionZ |
| Yaw (rad) | Yaw (deg) | RotationY (deg) |

El bridge ROS2 convierte automáticamente las coordenadas locales de TF a globales (sumando el offset de spawn de Gazebo) y aplica un offset de altura para coincidir con el nivel del suelo de Unity.

## Mapa Compartido: Dual Rooms

Se ha incluido un mapa base métricamente idéntico para verificar sincronización:
```
    Room A (4×4m)      Pasillo (2×1.5m)      Room B (4×4m)
   x=-5        x=-1   x=-1       x=1   x=1          x=5
   ┌────────────┐                       ┌────────────┐
   │            │ y=0.75 ┌──────┐ y=0.75│            │ y=2
   │            ├────────┤      ├───────┤            │
   │  Robot ★   │        │      │       │            │
   │  (-3, 0)   ├────────┤      ├───────┤            │
   │            │y=-0.75 └──────┘y=-0.75│            │ y=-2
   └────────────┘                       └────────────┘
```
- Paredes de 0.15m de grosor y 2.5m de alto.
- El script de Unity (`DualRoomsBuilder.cs`) lee el mundo actual de Gazebo desde `/tmp/dualsim_world.txt` para saber si debe generar las paredes al inicio de la simulación.

El bridge ROS2 convierte automáticamente quaternion→yaw en grados.
Unity ya trabaja en grados (eulerAngles.y).
