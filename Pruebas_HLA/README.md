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

---

# HLA Tests

This directory contains basic examples of HLA (High Level Architecture) communication using `jpype` to interact with the RTI (Pitch pRTI 1516e). The goal is to mimic the ROS2 Publisher and Subscriber structure for HLA.

## Content

### [chatter.py](file:///home/vicen/ISDEFE/Pruebas_HLA/chatter.py)
A simple "Publisher" federate.
- Creates (if it doesn't exist) and joins the `ChatterFed` federation.
- Publishes objects of the `Communication` class with the `Message` attribute.
- Sends "Hello World: X" messages periodically.

### [listener.py](file:///home/vicen/ISDEFE/Pruebas_HLA/listener.py)
A simple "Subscriber" federate.
- Joins the `ChatterFed` federation.
- Subscribes to the `Communication` class.
- Receives and decodes messages sent by the `chatter`.

### [chatter.xml](file:///home/vicen/ISDEFE/Pruebas_HLA/chatter.xml)
The FOM (Federation Object Model) defining the data structure for the federation (`Communication` Class, `Message` Attribute).

## How to run

Two terminals are required. Make sure you have configured the environment variable or the path to the RTI JAR (defined in the first lines of the Python scripts). Also, the RTI must be running for the Federation to be created and Federates to join.

**Terminal 1 (Publisher):**
```bash
python3 chatter.py
```

**Terminal 2 (Subscriber):**
```bash
python3 listener.py
```
