using UnityEngine;
using HLA;
using System.IO;

public class HlaNetworkManager : MonoBehaviour
{
    public static HlaNetworkManager Instance { get; private set; }

    [Header("HLA Settings")]
    public string federationName = "Test_Federation";
    public string federateName = "UnityClient";
    public string fomFileName = "BoxFOM.xml";

    [Tooltip("Ruta absoluta al FOM (si se deja vacío, busca en Assets/FOM/ y StreamingAssets/)")]
    public string fomAbsolutePath = "";

    [Header("Time Management")]
    [Tooltip("Lookahead in seconds. Lower = higher priority. Viewers use half this value.")]
    public double baseLookahead = 0.05; // 50ms base lookahead
    
    [Tooltip("Time step for simulation updates (0.0167 = 60fps, 0.033 = 30fps)")]
    public double timeStep = 0.0167; // ~60 FPS

    public bool IsConnected { get; private set; } = false;
    public bool IsTimeManaged { get; private set; } = false;
    public double CurrentLogicalTime { get; private set; } = 0.0;
    
    private FederateRole _myRole;
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
        // 1. Prioridad: ruta absoluta (evita problemas con espacios en paths)
        string fomPath = "";
        if (!string.IsNullOrEmpty(fomAbsolutePath) && File.Exists(fomAbsolutePath))
        {
            fomPath = fomAbsolutePath;
            Debug.Log($"[HLA Manager] Usando FOM desde ruta absoluta: {fomPath}");
        }
        else
        {
            // 2. Buscar en Assets/FOM/
            fomPath = Path.Combine(Application.dataPath, "FOM", fomFileName);
            // 3. Fallback: StreamingAssets
            if (!File.Exists(fomPath))
            {
                fomPath = Path.Combine(Application.streamingAssetsPath, fomFileName);
            }
        }
        if (!File.Exists(fomPath))
        {
            Debug.LogError($"[HLA Manager] CRITICAL: FOM file not found at {fomPath}. Aborting Connect to prevent crash.");
            return;
        }

        bool isSender = FindObjectOfType<HlaPlayerSender>(true) != null;  // true = includeInactive
        bool isViewer = FindObjectOfType<HlaCoordinateViewer>(true) != null;  // true = includeInactive

        string role = "Client";
        if (isSender && isViewer)
        {
            role = "Hybrid";
            _myRole = FederateRole.Hybrid;
        }
        else if (isViewer)  // Check Viewer FIRST for priority
        {
            role = "Viewer";
            _myRole = FederateRole.Viewer;
        }
        else if (isSender)
        {
            role = "Sender";
            _myRole = FederateRole.Sender;
        }
        else
        {
            role = "Client";
            _myRole = FederateRole.Sender; // Default
        }
        
        Debug.Log($"[HLA Manager] Role detection: isSender={isSender}, isViewer={isViewer}, finalRole={role}");

        // Use a unique FEDERATE name to avoid conflicts if multiple instances run.
        string uniqueFederateName = $"{federateName}_{role}_{System.Guid.NewGuid().ToString().Substring(0, 5)}";
        Debug.Log($"[HLA Manager] Connecting as {uniqueFederateName}...");
        
        try
        {
            // 1. Connect
            bool success = HlaInterface.Connect(federationName, uniqueFederateName, fomPath);
            
            if (!success)
            {
                 Debug.LogError($"[HLA Manager] FAILED to connect as {uniqueFederateName}. Check C++ plugin logs (Debug.Log) for details.");
                 IsConnected = false;
                 return;
            }
            
            // 2. Configure capabilities
            // Senders MUST publish
            if (isSender || role == "Client")
            {
                HlaInterface.PublishUnit();
                Debug.Log("[HLA Manager] Role: Publishing enabled.");
            }
            
            // BOTH Senders and Viewers should subscribe in the new unified model
            if (isViewer || isSender || role == "Client")
            {
                HlaInterface.SubscribeUnit();
                Debug.Log("[HLA Manager] Role: Subscribing enabled.");
            }
            
            IsConnected = true;
            Debug.Log($"[HLA Manager] Connected successfully as {uniqueFederateName}.");
            
            // 3. Enable Time Management (ALWAYS ACTIVE)
            EnableTimeManagementForRole();
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[HLA Manager] Crash/Error during Connect: {e.Message}");
        }
    }

    void EnableTimeManagementForRole()
    {
        // Viewers have lower lookahead = higher temporal priority
        double effectiveLookahead = _myRole == FederateRole.Viewer ? baseLookahead * 0.5 : baseLookahead;
        
        Debug.Log($"[HLA Time] Enabling Time Management as {_myRole} with lookahead {effectiveLookahead}s");
        
        try
        {
            HlaInterface.EnableTimeManagement((int)_myRole, effectiveLookahead);
            IsTimeManaged = true;
            Debug.Log($"[HLA Time] Time Management ENABLED. Starting logical time: {HlaInterface.GetCurrentLogicalTime()}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[HLA Time] Failed to enable Time Management: {e.Message}");
            IsTimeManaged = false;
        }
    }

    void Update()
    {
        if (!IsConnected) return;
        
        // Process RTI callbacks every frame
        HlaInterface.EvokeCallbacks(0.001); // ~1ms max
        
        // Update current logical time
        if (IsTimeManaged)
        {
            CurrentLogicalTime = HlaInterface.GetCurrentLogicalTime();
        }
    }

    /// <summary>
    /// Request time advance and check if granted. Call this before updating simulation state.
    /// </summary>
    /// <param name="deltaTime">Unity delta time or fixed time step</param>
    /// <returns>True if time advance was granted and simulation can proceed</returns>
    public bool RequestAndWaitTimeAdvance(double deltaTime)
    {
        if (!IsTimeManaged)
        {
            return true; // No time management, always proceed
        }
        
        // If we're not waiting for a grant, request a new advance
        if (!_pendingTimeAdvance)
        {
            HlaInterface.RequestTimeAdvance(timeStep);
            _pendingTimeAdvance = true;
        }
        
        // Process callbacks to receive grant
        HlaInterface.EvokeCallbacks(0.005); // ~5ms to process
        
        // Check if granted
        if (HlaInterface.IsTimeAdvanceGranted())
        {
            _pendingTimeAdvance = false;
            CurrentLogicalTime = HlaInterface.GetCurrentLogicalTime();
            return true;
        }
        
        // Not yet granted, skip this frame's simulation update
        return false;
    }

    void OnDestroy()
    {
        // Minimal cleanup - avoid calling Disconnect which can crash Unity
        if (IsConnected)
        {
            try
            {
                // Disable Time Management first if active
                if (IsTimeManaged)
                {
                    HlaInterface.DisableTimeManagement();
                    IsTimeManaged = false;
                }
                
                // Process pending callbacks
                HlaInterface.EvokeCallbacks(0.05);
                
                // Disconnect (Now safe)
                HlaInterface.Disconnect();
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[HLA Manager] OnDestroy disconnect warning: {e.Message}");
            }
            IsConnected = false;
        }
        
        // Clear singleton reference
        if (Instance == this)
        {
            Instance = null;
        }
    }

    void OnGUI()
    {
        if (!IsConnected) return;
        
        // Display Time Management status
        if (IsTimeManaged)
        {
            GUI.color = Color.green;
            GUI.Label(new Rect(Screen.width - 250, 10, 240, 20), $"HLA Time: {CurrentLogicalTime:F3}s");
            GUI.Label(new Rect(Screen.width - 250, 30, 240, 20), $"Role: {_myRole}");
        }
    }
}
