# Scripts — Pruebas ROS# y HLA-Unity

Documentación detallada de todos los scripts C# que componen la integración **HLA (High Level Architecture)** con **Unity** y **ROS# (ROS Sharp)**.

---

## Índice

1. [Arquitectura General](#arquitectura-general)
2. [HlaInterface.cs](#hlainterface)
3. [HlaNetworkManager.cs](#hlanetworkmanager)
4. [HlaPlayerSender.cs](#hlaplayersender)
5. [HlaCoordinateViewer.cs](#hlacoordinateviewer)
6. [Cmd_vel.cs](#cmd_vel)
7. [PlayerController_Bridge.cs](#playercontroller_bridge)
8. [PauseMenuController.cs](#pausemenucontroller)
9. [RandomPosition.cs](#randomposition)
10. [Flujo de Vida HLA Completo](#flujo-de-vida-hla-completo)

---

## Arquitectura General

El sistema conecta una o varias instancias de Unity a una **federación HLA** gestionada por un RTI (Run-Time Infrastructure, e.g. Pitch pRTI). Cada instancia de Unity actúa como un **federado** que puede asumir uno de los siguientes roles:

| Rol | Descripción |
|-----|-------------|
| **Sender** | Publica la posición/rotación del jugador local y recibe las de otros federados. |
| **Viewer** | Solo recibe y visualiza las posiciones publicadas por otros federados. |
| **Hybrid** | Combina Sender y Viewer. Publica la posición local y recibe las remotas. |

La comunicación con el RTI se realiza a través de un **plugin nativo C++** (`hla_plugin.dll` / `hla_plugin.so`) cuyas funciones se importan vía P/Invoke en `HlaInterface.cs`.

```
┌────────────────────────────────────────────────────────┐
│                    Unity (C#)                          │
│                                                        │
│  HlaNetworkManager (Singleton)                         │
│    ├── Gestiona conexión/desconexión                   │
│    ├── Gestiona Time Management                        │
│    └── Procesa callbacks del RTI cada frame            │
│                                                        │
│  HlaPlayerSender          HlaCoordinateViewer          │
│    ├── Publica posición     ├── Recibe posiciones      │
│    ├── Recibe remotos       ├── Instancia modelos 3D   │
│    └── Señal desconexión    └── Limpieza automática    │
│                                                        │
│  Cmd_vel (ROS#)           PlayerController_Bridge      │
│    └── /cmd_vel → mov.      └── WASD → movimiento      │
│                                                        │
│  PauseMenuController      RandomPosition               │
│    └── ESC → Menú            └── Spawn aleatorio       │
│                                                        │
│              ┌──────────────┐                          │
│              │ HlaInterface │ (P/Invoke → C++ plugin)  │
│              └──────┬───────┘                          │
└─────────────────────┼──────────────────────────────────┘
                      │
              ┌───────▼───────┐
              │   RTI (pRTI)  │
              │  Federación   │
              └───────────────┘
```

---

## HlaInterface

**Archivo:** `HlaInterface.cs`

Capa de **interoperabilidad (P/Invoke)** que expone las funciones del plugin nativo C++ `hla_plugin` a C#. No contiene lógica de negocio; actúa exclusivamente como puente entre Unity y el RTI.

### Enumeración `FederateRole`

```csharp
public enum FederateRole
{
    Sender = 0,   // Federado que envía datos
    Viewer = 1,   // Federado que solo recibe
    Hybrid = 2    // Envía y recibe
}
```

Define la prioridad temporal de cada rol. Los **Viewers** usan un lookahead menor (mayor prioridad temporal) para reducir la latencia de visualización.

### Estructura `BoxData`

```csharp
[StructLayout(LayoutKind.Sequential)]
public struct BoxData
{
    public int id;           // ID único del objeto en la federación
    public float positionX;  // Posición X (eje X de Unity)
    public float positionY;  // Posición Y (mapeada al eje Z de Unity)
    public float rotationY;  // Rotación en el eje Y (grados)
    public float positionZ;  // Posición Z (mapeada al eje Y de Unity / altura)
}
```

Representa un objeto (unidad/jugador) en la federación HLA. El layout `Sequential` garantiza compatibilidad binaria con la estructura C++ equivalente.

> **Nota sobre ejes:** El mapeo de coordenadas entre ROS/HLA (X-forward, Y-left, Z-up) y Unity (X-right, Y-up, Z-forward) se realiza durante la lectura/escritura de datos.

### Estructura `TimeManagementState`

```csharp
[StructLayout(LayoutKind.Sequential)]
public struct TimeManagementState
{
    public bool isTimeRegulating;      // ¿Este federado influye en el tiempo?
    public bool isTimeConstrained;     // ¿Este federado se restringe al tiempo?
    public bool timeAdvanceGranted;    // ¿Se concedió el último avance de tiempo?
    public double currentLogicalTime;  // Tiempo lógico actual (segundos)
    public double lookahead;           // Valor de lookahead configurado
}
```

### Funciones Importadas

#### Conexión y Gestión de Objetos

| Función | Descripción |
|---------|-------------|
| `Connect(federation, federate, fomPath)` → `bool` | Conecta al RTI, crea/une a la federación con el FOM especificado. |
| `Disconnect()` | Desconecta del RTI, dimite el federado y destruye la federación si es el último. |
| `PublishUnit()` | Declara intención de publicar atributos de la clase `Unit` del FOM. |
| `SubscribeUnit()` | Declara intención de recibir atributos de la clase `Unit` del FOM. |
| `CreateUnit()` → `int` | Registra una nueva instancia de `Unit` en la federación. Devuelve su ID. |
| `UpdateUnit(BoxData)` | Actualiza los atributos de posición/rotación en el RTI. |
| `GetBoxes(out count)` → `IntPtr` | Recupera un puntero al array de objetos `Unit` discovereados. |

#### Time Management

| Función | Descripción |
|---------|-------------|
| `EnableTimeManagement(role, lookahead)` | Habilita Time Regulating y Time Constrained con el lookahead dado. |
| `DisableTimeManagement()` | Deshabilita la gestión temporal del federado. |
| `RequestTimeAdvance(timeStep)` | Solicita avanzar el tiempo lógico en `timeStep` segundos. |
| `IsTimeAdvanceGranted()` → `bool` | Comprueba si el RTI concedió el último avance solicitado. |
| `GetCurrentLogicalTime()` → `double` | Devuelve el tiempo lógico actual concedido. |
| `GetTimeManagementState()` → `TimeManagementState` | Devuelve el estado completo de la gestión temporal. |
| `EvokeCallbacks(seconds)` | Procesa callbacks del RTI durante hasta `seconds` segundos. |

---

## HlaNetworkManager

**Archivo:** `HlaNetworkManager.cs`

**Componente Singleton** (`DontDestroyOnLoad`) que centraliza toda la gestión del ciclo de vida HLA. Solo debe existir una instancia en la escena.

### Propiedades Públicas Configurables (Inspector)

| Propiedad | Tipo | Valor por defecto | Descripción |
|-----------|------|-------------------|-------------|
| `federationName` | `string` | `"Test_Federation"` | Nombre de la federación HLA. |
| `federateName` | `string` | `"UnityClient"` | Prefijo del nombre del federado. |
| `fomFileName` | `string` | `"BoxFOM.xml"` | Nombre del archivo FOM (Federation Object Model). |
| `baseLookahead` | `double` | `0.05` (50ms) | Lookahead base en segundos. |
| `timeStep` | `double` | `0.0167` (~60 FPS) | Paso de tiempo para la simulación. |

### Propiedades de Estado (Solo Lectura)

| Propiedad | Descripción |
|-----------|-------------|
| `IsConnected` | `true` si la conexión al RTI está activa. |
| `IsTimeManaged` | `true` si la gestión temporal está habilitada. |
| `CurrentLogicalTime` | Tiempo lógico actual de la federación (en segundos). |

### Ciclo de Vida Detallado

#### 1. Inicialización (`Awake`)

- Implementa el patrón **Singleton**: si ya existe una instancia, destruye la duplicada.
- Marca el GameObject con `DontDestroyOnLoad` para persistir entre escenas.

#### 2. Conexión (`Start`)

```
Buscar FOM → Detectar Rol → Conectar → Publicar/Suscribir → Habilitar Time Management
```

1. **Búsqueda del FOM**: Busca `BoxFOM.xml` primero en `Assets/FOM/`, después en `StreamingAssets/`. Si no lo encuentra, aborta con un error crítico.

2. **Detección automática de rol**: Inspecciona la escena buscando componentes `HlaPlayerSender` y `HlaCoordinateViewer` (incluyendo GameObjects inactivos):
   - **Sender + Viewer** → `Hybrid`
   - **Solo Viewer** → `Viewer`
   - **Solo Sender** → `Sender`
   - **Ninguno** → `Sender` (por defecto)

3. **Nombre único de federado**: Genera un nombre único combinando el prefijo, el rol y un fragmento de GUID aleatorio:
   ```
   UnityClient_Sender_a3f2b
   ```
   Esto permite ejecutar múltiples instancias sin conflicto de nombres.

4. **Conexión al RTI**: Llama a `HlaInterface.Connect()` con la federación, el nombre único y la ruta del FOM.

5. **Publicación y Suscripción**:
   - Los **Senders** publican (`PublishUnit()`) para enviar datos.
   - **Todos los roles** suscriben (`SubscribeUnit()`) para recibir datos de otros federados.

6. **Habilitación del Time Management**: Llama a `EnableTimeManagementForRole()`.

#### 3. Time Management (`EnableTimeManagementForRole`)

Configura el federado como **Time Regulating** (influye en el avance temporal de otros) y **Time Constrained** (se restringe al tiempo lógico global).

- **Lookahead efectivo**:
  - `Viewer`: `baseLookahead × 0.5` → Mayor prioridad temporal, menor latencia.
  - `Sender / Hybrid`: `baseLookahead` → Lookahead completo.

> **¿Qué es el lookahead?** Es el mínimo intervalo de tiempo futuro para el que un federado puede enviar eventos. Un lookahead menor permite enviar eventos más cercanos al tiempo actual, dando mayor prioridad temporal al federado.

#### 4. Bucle Principal (`Update` — cada frame)

```csharp
if (!IsConnected) return;
HlaInterface.EvokeCallbacks(0.001); // Procesar callbacks del RTI (~1ms máx.)
CurrentLogicalTime = HlaInterface.GetCurrentLogicalTime(); // Actualizar tiempo
```

- **`EvokeCallbacks(0.001)`**: Procesa mensajes pendientes del RTI (descubrimiento de objetos, actualizaciones de atributos, concesiones de tiempo). Limitado a ~1ms para no bloquear el frame de Unity.

#### 5. Avance de Tiempo (`RequestAndWaitTimeAdvance`)

Método público que los componentes `HlaPlayerSender` y `HlaCoordinateViewer` llaman antes de actualizar su estado:

```
¿Time Management activo? ─No─→ Proceder sin restricción
         │ Sí
         ▼
¿Avance pendiente? ─No─→ Solicitar avance (RequestTimeAdvance)
         │ Sí
         ▼
Procesar callbacks (EvokeCallbacks ~5ms)
         │
         ▼
¿Avance concedido? ─Sí─→ Actualizar tiempo → return true (proceder)
         │ No
         ▼
return false (skip frame)
```

- Si no hay gestión temporal activa, siempre devuelve `true`.
- Si hay una solicitud de avance pendiente, espera la concesión del RTI.
- Si el RTI no concede el avance (porque otro federado aún no ha alcanzado ese tiempo), el frame se salta y la simulación espera.

#### 6. Desconexión (`OnDestroy`)

1. Deshabilita Time Management (`DisableTimeManagement`).
2. Procesa callbacks pendientes (`EvokeCallbacks(0.05)` — 50ms).
3. Desconecta del RTI (`Disconnect`).
4. Limpia la referencia singleton.

#### 7. Interfaz Visual (`OnGUI`)

Muestra en la esquina superior derecha:
- **Tiempo lógico HLA** actual en segundos.
- **Rol** del federado (Sender/Viewer/Hybrid).

---

## HlaPlayerSender

**Archivo:** `HlaPlayerSender.cs`

Script que se asigna al **jugador local**. Publica la posición y rotación del GameObject al que está adjunto y recibe/visualiza las posiciones de jugadores remotos.

### Funcionalidad

#### Publicación del Jugador Local

1. **Creación de unidad** (`CreateMyUnit`): En el primer frame tras la conexión, registra una nueva instancia de `Unit` en la federación mediante `HlaInterface.CreateUnit()`. Obtiene un ID único.

2. **Actualización continua** (`UpdateLocalPlayer`): Cada frame (tras concesión del Time Advance), envía la posición y rotación del Transform al RTI:
   ```csharp
   myBoxData.positionX = transform.position.x;     // Unity X → HLA X
   myBoxData.positionY = transform.position.z;     // Unity Z → HLA Y
   myBoxData.rotationY = transform.eulerAngles.y;  // Rotación Y (yaw)
   myBoxData.positionZ = transform.position.y;     // Unity Y → HLA Z (altura)
   ```

#### Recepción de Jugadores Remotos

1. **Obtención de datos** (`UpdateRemoteBoxes`): Recupera todos los objetos `Unit` descubiertos por el RTI mediante `HlaInterface.GetBoxes()`.

2. **Filtrado** (`receivedBoxesFromRTI`):
   - **Self-filter**: Excluye el propio ID del jugador local.
   - **Señal de desconexión**: Excluye objetos con `positionX ≤ -9990` (señal de desconexión, ver más abajo).

3. **Visualización 3D** (`Sync3DModels`): Para cada jugador remoto activo:
   - **Nuevo**: Crea un GameObject wrapper con el prefab del jugador instanciado como hijo. La jerarquía wrapper→modelo evita problemas de gimbal lock al aplicar solo rotación en Y al wrapper, preservando la rotación baked del modelo importado.
   - **Existente**: Actualiza posición y rotación del wrapper.
   - **Desaparecido**: Destruye el GameObject al detectar que un ID ya no está presente.

#### Señal de Desconexión

Al cerrar la aplicación (`OnApplicationQuit`) o destruir el componente (`OnDestroy`), envía una posición especial `(-9999, -9999, 0, 0)` antes de desconectar. Esto permite a otros federados identificar y eliminar inmediatamente al jugador que se ha desconectado, en lugar de esperar un timeout.

```csharp
private const float DISCONNECT_SIGNAL = -9999.0f;
```

#### Interfaz Visual (`OnGUI`)

Muestra:
- ID local y posición/rotación del jugador.
- Lista de jugadores remotos visibles con sus coordenadas.

---

## HlaCoordinateViewer

**Archivo:** `HlaCoordinateViewer.cs`

Script para instancias que **solo visualizan** (no publican posición propia). Ideal para observadores o pantallas de monitorización.

### Funcionalidad

#### Recepción y Visualización

1. **Time Management**: Antes de actualizar, solicita avance de tiempo al Manager. Si no se concede, salta el frame.

2. **UpdateBoxes**: Recupera los objetos `Unit` del RTI y los almacena en `receivedBoxes`, filtrando la señal de desconexión (posición ≤ -9990).

3. **Sync3DModels**: Idéntico al de `HlaPlayerSender`:
   - Crea wrappers con modelos 3D para nuevos IDs.
   - Actualiza posición/rotación de existentes.
   - Destruye los que desaparecen.

   **Mapeo de coordenadas HLA → Unity**:
   ```csharp
   position = new Vector3(data.positionX, data.positionZ, data.positionY);
   //                      HLA X → Unity X  HLA Z → Unity Y  HLA Y → Unity Z
   rotation = Quaternion.Euler(0f, data.rotationY, 0f);
   //                             Solo rotación Yaw
   ```

#### Interfaz Visual (`OnGUI`)

Muestra:
- Estado de conexión y número de objetos activos.
- Lista detallada de todos los objetos con ID, coordenadas y rotación.

#### Diferencias respecto a `HlaPlayerSender`

| Aspecto | `HlaPlayerSender` | `HlaCoordinateViewer` |
|---------|--------------------|-----------------------|
| Publica datos | ✅ Sí | ❌ No |
| Crea unidad propia | ✅ `CreateUnit()` | ❌ No |
| Filtra self | ✅ Por ID | No necesario |
| Envía señal de desconexión | ✅ `-9999` | ❌ No |
| Lookahead | `baseLookahead` | `baseLookahead × 0.5` |

---

## Cmd_vel

**Archivo:** `Cmd_vel.cs`

Suscriptor de **ROS#** que recibe mensajes de tipo `geometry_msgs/Twist` del tópico ROS `/cmd_vel` y los aplica como movimiento al GameObject en Unity.

### Parámetros Configurables

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `TopicName` | `string` | `"/cmd_vel"` | Tópico ROS al que suscribirse. |
| `linearScale` | `float` | `1.0` | Factor de escala para velocidad lineal. |
| `angularScale` | `float` | `1.0` | Factor de escala para velocidad angular. |
| `useRosToUnityMapping` | `bool` | `true` | Habilitar mapeo automático de ejes ROS→Unity. |
| `invertAngular` | `bool` | `false` | Invertir dirección de rotación angular. |

### Mapeo de Ejes ROS → Unity

```
ROS:     X = forward,  Y = left,  Z = up
Unity:   X = right,    Y = up,    Z = forward

Mapeo:   ROS.x → Unity.z (forward)
         ROS.y → Unity.x (lateral, invertido)
         Angular.z → Rotación en Y de Unity (invertido)
```

### Flujo de Datos

```
ROS2 ─────► rosbridge_server ─────► ROS# (WebSocket) ─────► Cmd_vel.cs
  /cmd_vel     (JSON)              (deserialización)      (movimiento Unity)
```

1. **Recepción** (`ReceiveMessage`): Se ejecuta en el hilo de ROS#. Almacena velocidades en variables protegidas por `lock`.
2. **Aplicación** (`Update`): En el hilo principal de Unity, aplica:
   - **Traslación local** (`transform.Translate`) con la velocidad lineal escalada.
   - **Rotación en Y** (`transform.Rotate`) con la velocidad angular convertida de rad/s a °/s.

### Thread Safety

El script usa un `object messageLock` para proteger las variables compartidas entre el hilo de ROS# (recepción) y el hilo principal de Unity (Update).

---

## PlayerController_Bridge

**Archivo:** `PlayerController_Bridge.cs`

Controlador de movimiento local con **teclado** usando el **New Input System** de Unity. Mueve al jugador con WASD o las flechas.

### Parámetros

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `speed` | `float` | `5.0` | Velocidad de avance/retroceso (unidades/s). |
| `rotationSpeed` | `float` | `120.0` | Velocidad de giro (°/s). |

### Funcionamiento

- Usa `FixedUpdate` para movimiento basado en física (`Rigidbody`).
- **W / ↑**: Avanzar en la dirección actual (`transform.forward`).
- **S / ↓**: Retroceder.
- **A / ←**: Girar a la izquierda.
- **D / →**: Girar a la derecha.
- **Inversión al retroceder**: Al ir hacia atrás, la dirección de giro se invierte para simular movimiento realista (como un vehículo).

### Requisitos

- Requiere un componente `Rigidbody` en el mismo GameObject.
- Requiere el paquete **Input System** de Unity instalado.

---

## PauseMenuController

**Archivo:** `PauseMenuController.cs`

Controla un **menú de pausa** que se activa con la tecla **ESC**. Gestiona la desconexión segura de HLA al volver al menú principal.

### Parámetros

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `menuCanvas` | `GameObject` | — | Referencia al Canvas del menú de pausa. |
| `mainMenuSceneName` | `string` | `"Init"` | Nombre de la escena del menú principal. |

### Funciones

#### `ToggleMenu()`
- Alterna visibilidad del menú.
- Desbloquea/bloquea el cursor del ratón.
- Compatible con **New Input System** y **Legacy Input** (fallback).

#### `GoToMainMenu()`
Secuencia de desconexión segura:
1. **Deshabilitar Time Management** (`DisableTimeManagement`).
2. **Procesar callbacks** pendientes (50ms).
3. **Desconectar del RTI** (`Disconnect`).
4. **Destruir** el singleton `HlaNetworkManager`.
5. **Cargar escena** del menú principal.

> Este método es seguro de llamar en cualquier momento gracias a que el plugin C++ fue corregido para manejar la desconexión limpiamente.

#### `QuitGame()`
- Llama a `Application.Quit()`.
- Esto dispara `OnApplicationQuit` en `HlaPlayerSender`, que envía la señal de desconexión `-9999` antes de cerrar.
- En el Editor de Unity, detiene el Play Mode.

---

## RandomPosition

**Archivo:** `RandomPosition.cs`

Script sencillo que posiciona el GameObject en una **coordenada aleatoria** dentro de un área configurable al iniciarse.

### Parámetros

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `minX` / `maxX` | `float` | `-10` / `10` | Rango del eje X para el spawn. |
| `minZ` / `maxZ` | `float` | `-10` / `10` | Rango del eje Z para el spawn. |
| `fixedY` | `float` | `0.51` | Altura fija (si `useCurrentY = false`). |
| `useCurrentY` | `bool` | `true` | Usar la Y actual del objeto o la fijada. |

### Uso

Se ejecuta solo en `Start()`. Ideal para posicionar jugadores en puntos de spawn aleatorios al iniciar una partida multijugador.

---

## Flujo de Vida HLA Completo

### 1. Conexión y Setup

```
Unity Scene Load
      │
      ▼
HlaNetworkManager.Awake()
      │  → Singleton + DontDestroyOnLoad
      ▼
HlaNetworkManager.Start()
      │  → Buscar FOM
      │  → Detectar rol (Sender/Viewer/Hybrid)
      │  → Connect(federation, federate, fomPath)
      │  → PublishUnit() si es Sender/Hybrid
      │  → SubscribeUnit() para todos
      │  → EnableTimeManagement(role, lookahead)
      ▼
HlaPlayerSender.Update() [primer frame]
      │  → CreateUnit() → obtiene ID único
      ▼
      ✅ SISTEMA OPERATIVO
```

### 2. Bucle Principal (cada frame)

```
HlaNetworkManager.Update()
      │  → EvokeCallbacks(0.001)      ← Procesa mensajes del RTI
      │  → Actualiza CurrentLogicalTime
      ▼
HlaPlayerSender.Update() / HlaCoordinateViewer.Update()
      │  → RequestAndWaitTimeAdvance()
      │       ├── RequestTimeAdvance(timeStep) → Solicita avanzar
      │       ├── EvokeCallbacks(0.005)        → Espera concesión
      │       └── IsTimeAdvanceGranted()?
      │            ├── true  → Continuar UPDATE
      │            └── false → Skip frame
      ▼
[Si concedido]:
      ├── Sender: UpdateLocalPlayer() → UpdateUnit(boxData)
      ├── Sender: UpdateRemoteBoxes() → GetBoxes() → Sync3DModels()
      └── Viewer: UpdateBoxes() → GetBoxes() → Sync3DModels()
```

### 3. Desconexión

```
Salir / Volver al menú
      │
      ▼
HlaPlayerSender.OnDestroy() / OnApplicationQuit()
      │  → SendDisconnectSignal() → UpdateUnit(-9999, -9999, ...)
      ▼
HlaNetworkManager.OnDestroy()
      │  → DisableTimeManagement()
      │  → EvokeCallbacks(0.05)
      │  → Disconnect()
      │  → Limpieza singleton
      ▼
      ✅ DESCONEXIÓN COMPLETA
```

### 4. Sincronización Temporal (Time Advance Grant)

```
Federado A (Sender)         RTI (pRTI)           Federado B (Viewer)
      │                        │                        │
      ├─ RequestTimeAdvance ──►│                        │
      │    (t + 0.0167)        │◄── RequestTimeAdvance ─┤
      │                        │    (t + 0.0167)        │
      │                        │                        │
      │                        │ [Calcula LBTS]         │
      │                        │ (Lowest Boundary       │
      │                        │  Time Stamp)           │
      │                        │                        │
      │◄── TimeAdvanceGrant ───┤─── TimeAdvanceGrant ──►│
      │     (t + 0.0167)       │    (t + 0.0167)        │
      │                        │                        │
      ├─ UpdateUnit(pos) ─────►│────► reflectValues ───►│
      │                        │                        │
      ▼                        ▼                        ▼
   t = t + 0.0167          LBTS avanza            t = t + 0.0167
```

- El **RTI** no concede un avance de tiempo a ningún federado hasta que **todos** hayan solicitado avanzar al menos hasta ese punto.
- Esto garantiza **causalidad**: un federado nunca recibe eventos del futuro relativo a su tiempo lógico.
- El **lookahead** limita cuán cerca del tiempo actual puede un federado enviar eventos, permitiendo al RTI optimizar la concesión de avances.