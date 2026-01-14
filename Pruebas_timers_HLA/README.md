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
