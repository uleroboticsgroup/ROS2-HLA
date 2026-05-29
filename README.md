# Documentación del Proyecto ROS2-HLA

Este es el directorio raíz del entorno de trabajo para la integración de ROS 2 y HLA (High Level Architecture). A continuación se describe el contenido de cada directorio:

**Autor:** Vicente Barreiro Serrano  
**Supervisión:** Vicente Matellán Olivera  
*Grupo de Robótica de la Universidad de León*


## Índice de Directorios

- **[Pruebas_HLA](Pruebas_HLA)**  
  Ejemplos básicos de funcionamiento de la HLA (Chatter/Listener) para verificar la conectividad y el uso básico de la RTI.

- **[Pruebas_timers_HLA](Pruebas_timers_HLA)**  
  Ejemplos avanzados utilizando la gestión de tiempo de HLA (Time Management) y sincronización. Incluye simulaciones de "Sine Wave" sincronizadas.

- **[ros2_hla_ws](ros2_hla_ws)**  
  Workspace de ROS 2 que contiene el puente `ROS2HLA_Bridge`. Para más información sobre el puente, véase el README del paquete [`ROS2HLA_Bridge`](ros2_hla_ws/src/ROS2HLA_Bridge/README.md).

- **[TrickHLA](TrickHLA)**  
  Recursos relacionados con el framework TrickHLA de NASA.

- **[Documentacion](Documentacion)**  
  Documentación general adicional del proyecto.

- **[TurtleBot_DualSim](TurtleBot_DualSim)**  
  Workspace de ROS 2 (`ros2_ws`) que contiene el paquete `dual_sim_bridge`. Este puente actúa como un federado HLA que sincroniza bidireccionalmente la pose de un robot simulado en Gazebo con otros federados. Incluye soporte para sincronización de tiempo (Time Regulation/Constrained), mapeo de coordenadas Gazebo-Unity, selección de mundos compartidos (como `dual_rooms`) y aplicación de offset de alturas.

- **[Pruebas Unity](Pruebas%20Unity)**  
  Proyecto de Unity que actúa como el gemelo digital (Digital Twin) en el sistema de simulación dual. Se conecta a la federación HLA usando un plugin nativo de Pitch pRTI y scripts (`HlaPlayerSender.cs`, `HlaNetworkManager.cs`) para enviar su posición y representar entidades remotas (el TurtleBot de Gazebo). Incluye el sistema `DualRoomsBuilder.cs` para la generación dinámica de entornos que coinciden métricamente con la simulación de Gazebo.

---

# ROS2-HLA Project Documentation

This is the root directory of the workspace for the ROS 2 and HLA (High Level Architecture) integration. The content of each directory is described below:

**Author:** Vicente Barreiro Serrano  
**Supervision:** Vicente Matellán Olivera  
*Robotics Group of the University of León*


## Directory Index

- **[Pruebas_HLA](Pruebas_HLA)**  
  Basic examples of HLA operation (Chatter/Listener) to verify connectivity and basic RTI usage.

- **[Pruebas_timers_HLA](Pruebas_timers_HLA)**  
  Advanced examples using HLA Time Management and synchronization. Includes synchronized "Sine Wave" simulations.

- **[ros2_hla_ws](ros2_hla_ws)**  
  ROS 2 Workspace containing the `ROS2HLA_Bridge`. For more information on the bridge, see the [`ROS2HLA_Bridge` README](ros2_hla_ws/src/ROS2HLA_Bridge/README.md).

- **[TrickHLA](TrickHLA)**  
  Resources related to NASA's TrickHLA framework.

- **[Documentacion](Documentacion)**  
  Additional general project documentation.

- **[TurtleBot_DualSim](TurtleBot_DualSim)**  
  ROS 2 workspace (`ros2_ws`) containing the `dual_sim_bridge` package. This bridge acts as an HLA federate that bidirectionally synchronizes the pose of a simulated robot in Gazebo with other federates. It features time management support (Time Regulation/Constrained), Gazebo-Unity coordinate mapping, shared world selection (such as `dual_rooms`), and automatic height offset adjustments.

- **[Pruebas Unity](Pruebas%20Unity)**  
  Unity project acting as the digital twin in the dual simulation system. It connects to the HLA federation using a native Pitch pRTI plugin and custom scripts (`HlaPlayerSender.cs`, `HlaNetworkManager.cs`) to publish its position and visualize remote entities (the Gazebo TurtleBot). It includes the `DualRoomsBuilder.cs` system for generating dynamic environments that exactly match the metrics of the Gazebo simulation.
