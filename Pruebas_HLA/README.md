# Pruebas HLA

Este directorio contiene ejemplos básicos de comunicación HLA (High Level Architecture) utilizando `jpype` para interactuar con la RTI (Pitch pRTI 1516e). El objetivo es imitar la estructura de Publisher y Subscriber de ROS2 para HLA.

## Contenido

### [chatter.py](file:///home/vicen/ISDEFE/Pruebas_HLA/chatter.py)
Un federado "Publicador" simple.
- Crea (si no existe) y se une a la federación `ChatterFed`.
- Publica objetos de la clase `Communication` con el atributo `Message`.
- Envía mensajes "Hello World: X" periódicamente.

### [listener.py](file:///home/vicen/ISDEFE/Pruebas_HLA/listener.py)
Un federado "Suscriptor" simple.
- Se suscribe a la federación `ChatterFed`.
- Se suscribe a la clase `Communication`.
- Recibe y decodifica los mensajes enviados por el `chatter`.

### [chatter.xml](file:///home/vicen/ISDEFE/Pruebas_HLA/chatter.xml)
El FOM (Federation Object Model) que define la estructura de datos para la federación (Clase `Communication`, Atributo `Message`).

## Cómo ejecutar

Se requieren dos terminales. Asegúrate de tener configurada la variable de entorno o la ruta al JAR de la RTI (definida en las primeras líneas de los scripts Python). Además el RTI tiene que estar ejecutado para que la Federación pueda ser creada y los Federados puedan unirse.

**Terminal 1 (Publicador):**
```bash
python3 chatter.py
```

**Terminal 2 (Suscriptor):**
```bash
python3 listener.py
```
