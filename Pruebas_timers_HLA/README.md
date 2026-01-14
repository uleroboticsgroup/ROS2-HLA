# Pruebas Timers HLA

Este directorio contiene ejemplos de sincronización y gestión de tiempo (Time Management) en HLA. Muestra cómo coordinar federados para que avancen en el tiempo lógico de manera sincronizada. El objetivo es entender procesos de sincronización y gestión de tiempo en HLA usando jpype como intermediario entre Python y la RTI.

**Nota de Atribución:** Estos archivos están basados en el ejemplo del README de [TrickHLA](https://github.com/nasa/TrickHLA/tree/master?tab=readme-ov-file#running-an-example-simulation).

## Contenido

### [sine_publisher.py](file:///home/vicen/ISDEFE/Pruebas_timers_HLA/sine_publisher.py)
Federado **Time Regulating** (Regulador de Tiempo).
- Publica valores de una onda senoidal (`SineWave.value`) asociados a un tiempo lógico.
- Controla el avance del tiempo de la federación.
- Utiliza **Puntos de Sincronización** (`ReadyToStart`) para asegurar que todos los federados comiencen a la vez.

### [sine_listener.py](file:///home/vicen/ISDEFE/Pruebas_timers_HLA/sine_listener.py)
Federado **Time Constrained** (Restringido por Tiempo).
- Se suscribe a los valores de la onda senoidal.
- Su avance de tiempo está limitado por el `sine_publisher`. Solo avanza cuando recibe el Grant (TAG) de la RTI, asegurando que procesa los mensajes en el orden temporal correcto.

### [sine.xml](file:///home/vicen/ISDEFE/Pruebas_timers_HLA/sine.xml)
El FOM para este ejemplo.

## Cómo ejecutar

Es necesario ejecutar ambos scripts. El orden ideal es iniciar primero el publicador (que suele crear la federación) y luego el listener, aunque el código maneja la espera. También es importante que el RTI esté ejecutado para que la federación pueda ser creada y los federados puedan unirse.

**Terminal 1 (Publicador):**
```bash
python3 sine_publisher.py
```
*El script pedirá confirmación de usuario (Enter) una vez que se registre el punto de sincronización.*

**Terminal 2 (Listener):**
```bash
python3 sine_listener.py
```
*El listener esperará automáticamente al punto de sincronización.*

Una vez ambos estén corriendo, presiona **Enter** en la terminal del `sine_publisher.py` para iniciar la simulación sincronizada.

---

# HLA Timers Tests

This directory contains examples of synchronization and Time Management in HLA. It shows how to coordinate federates to advance in logical time in a synchronized way. The goal is to understand synchronization and time management processes in HLA using jpype as an intermediary between Python and the RTI.

**Attribution Note:** These files are based on the example in the README of [TrickHLA](https://github.com/nasa/TrickHLA/tree/master?tab=readme-ov-file#running-an-example-simulation).

## Content

### [sine_publisher.py](file:///home/vicen/ISDEFE/Pruebas_timers_HLA/sine_publisher.py)
**Time Regulating** Federate.
- Publishes sine wave values (`SineWave.value`) associated with a logical time.
- Controls the federation's time advancement.
- Uses **Synchronization Points** (`ReadyToStart`) to ensure all federates start at the same time.

### [sine_listener.py](file:///home/vicen/ISDEFE/Pruebas_timers_HLA/sine_listener.py)
**Time Constrained** Federate.
- Subscribes to the sine wave values.
- Its time advancement is constrained by the `sine_publisher`. It only advances when it receives the Grant (TAG) from the RTI, ensuring it processes messages in the correct temporal order.

### [sine.xml](file:///home/vicen/ISDEFE/Pruebas_timers_HLA/sine.xml)
The FOM for this example.

## How to run

It is necessary to run both scripts. The ideal order is to start the publisher (which usually creates the federation) first and then the listener, although the code handles waiting. It is also important that the RTI is running for the federation to be created and federates to join.

**Terminal 1 (Publisher):**
```bash
python3 sine_publisher.py
```
*The script will ask for user confirmation (Enter) once the synchronization point is registered.*

**Terminal 2 (Listener):**
```bash
python3 sine_listener.py
```
*The listener will automatically wait for the synchronization point.*

Once both are running, press **Enter** in the `sine_publisher.py` terminal to start the synchronized simulation.
