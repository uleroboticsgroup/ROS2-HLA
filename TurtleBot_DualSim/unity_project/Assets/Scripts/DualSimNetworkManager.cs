using UnityEngine;
using HLA;
using System.IO;

/// <summary>
/// Gestiona la conexión HLA para el sistema DualSim.
/// Se conecta a la federación "DualSimFed", publica y suscribe
/// la clase Box para intercambiar poses de robots.
/// 
/// CONFIGURACIÓN: Adjuntar a un GameObject vacío en la escena.
/// Asegurar que el FOM "DualSimFOM.xml" está en Assets/FOM/.
/// </summary>
public class DualSimNetworkManager : MonoBehaviour
{
    public static DualSimNetworkManager Instance { get; private set; }

    [Header("HLA Settings")]
    [Tooltip("Nombre de la federación (debe coincidir con Gazebo)")]
    public string federationName = "DualSimFed";
    [Tooltip("Nombre de este federado")]
    public string federateName = "UnityFederate";
    [Tooltip("Nombre del archivo FOM en Assets/FOM/")]
    public string fomFileName = "DualSimFOM.xml";

    [Header("Time Management")]
    [Tooltip("Lookahead en segundos")]
    public double baseLookahead = 0.05;
    [Tooltip("Paso de tiempo por frame (0.05 = 20Hz)")]
    public double timeStep = 0.05;

    public bool IsConnected { get; private set; } = false;
    public bool IsTimeManaged { get; private set; } = false;
    public double CurrentLogicalTime { get; private set; } = 0.0;

    private FederateRole _myRole = FederateRole.Hybrid;
    private bool _pendingTimeAdvance = false;

    void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(this.gameObject);
            return;
        }
        Instance = this;
        DontDestroyOnLoad(this.gameObject);
    }

    void Start()
    {
        // Buscar el FOM
        string fomPath = Path.Combine(Application.dataPath, "FOM", fomFileName);
        if (!File.Exists(fomPath))
        {
            fomPath = Path.Combine(Application.streamingAssetsPath, fomFileName);
        }
        if (!File.Exists(fomPath))
        {
            Debug.LogError($"[DualSim] FOM no encontrado en {fomPath}. Abortando conexión.");
            return;
        }

        // Nombre único para evitar conflictos
        string uniqueName = $"{federateName}_{System.Guid.NewGuid().ToString().Substring(0, 5)}";
        Debug.Log($"[DualSim] Conectando como '{uniqueName}' a federación '{federationName}'...");

        try
        {
            // 1. Conectar
            bool success = HlaInterface.Connect(federationName, uniqueName, fomPath);
            if (!success)
            {
                Debug.LogError($"[DualSim] FALLO al conectar como {uniqueName}.");
                return;
            }

            // 2. Publicar y suscribirse a Box
            HlaInterface.PublishUnit();
            HlaInterface.SubscribeUnit();
            Debug.Log("[DualSim] Publicación y suscripción habilitadas.");

            IsConnected = true;

            // 3. Activar gestión de tiempo (Hybrid)
            EnableTimeManagement();

        }
        catch (System.Exception e)
        {
            Debug.LogError($"[DualSim] Error durante conexión: {e.Message}");
        }
    }

    void EnableTimeManagement()
    {
        Debug.Log($"[DualSim] Activando Time Management como {_myRole} (lookahead={baseLookahead})");
        try
        {
            HlaInterface.EnableTimeManagement((int)_myRole, baseLookahead);
            IsTimeManaged = true;
            Debug.Log($"[DualSim] Time Management ACTIVO. Tiempo lógico: {HlaInterface.GetCurrentLogicalTime()}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[DualSim] Error activando Time Management: {e.Message}");
            IsTimeManaged = false;
        }
    }

    void Update()
    {
        if (!IsConnected) return;

        // Procesar callbacks RTI cada frame
        HlaInterface.EvokeCallbacks(0.001);

        if (IsTimeManaged)
        {
            CurrentLogicalTime = HlaInterface.GetCurrentLogicalTime();
        }
    }

    /// <summary>
    /// Solicita avance de tiempo y verifica si fue concedido.
    /// Llamar antes de actualizar el estado de la simulación.
    /// </summary>
    public bool RequestAndWaitTimeAdvance(double deltaTime)
    {
        if (!IsTimeManaged) return true;

        if (!_pendingTimeAdvance)
        {
            HlaInterface.RequestTimeAdvance(timeStep);
            _pendingTimeAdvance = true;
        }

        HlaInterface.EvokeCallbacks(0.005);

        if (HlaInterface.IsTimeAdvanceGranted())
        {
            _pendingTimeAdvance = false;
            CurrentLogicalTime = HlaInterface.GetCurrentLogicalTime();
            return true;
        }

        return false;
    }

    void OnDestroy()
    {
        if (IsConnected)
        {
            try
            {
                if (IsTimeManaged)
                {
                    HlaInterface.DisableTimeManagement();
                    IsTimeManaged = false;
                }
                HlaInterface.EvokeCallbacks(0.05);
                HlaInterface.Disconnect();
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[DualSim] Aviso al desconectar: {e.Message}");
            }
            IsConnected = false;
        }

        if (Instance == this) Instance = null;
    }

    void OnGUI()
    {
        if (!IsConnected) return;

        if (IsTimeManaged)
        {
            GUI.color = Color.green;
            GUI.Label(new Rect(Screen.width - 280, 10, 270, 20),
                $"HLA Time: {CurrentLogicalTime:F3}s | Federation: {federationName}");
            GUI.Label(new Rect(Screen.width - 280, 30, 270, 20),
                $"Role: {_myRole} | DualSim Active");
        }
    }
}
