using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.InputSystem;
using HLA;

public class PauseMenuController : MonoBehaviour
{
    [Header("UI References")]
    public GameObject menuCanvas; 

    [Header("Configuration")]
    public string mainMenuSceneName = "Init"; 
    
    private bool isPaused = false;

    void Start()
    {
        Debug.Log("[PauseMenu] Script INICIADO. Si no se ve esto, es que el script no está en la escena.");

        // Ensure menu is hidden at start
        if (menuCanvas != null)
        {
            menuCanvas.SetActive(false);
        }
        else
        {
            Debug.LogError("[PauseMenu] ALERTA: No has asignado el 'Menu Canvas' en el Inspector!");
        }
    }

    void Update()
    {
        // Check for New Input System
        if (Keyboard.current != null)
        {
            if (Keyboard.current.escapeKey.wasPressedThisFrame)
            {
                Debug.Log("[PauseMenu] TECLA ESC DETECTADA (New Input System).");
                ToggleMenu();
                return;
            }
        }
        
        // Fallback or "Old" Input System check (just in case 'Both' is active)
        try
        {
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                Debug.Log("[PauseMenu] TECLA ESC DETECTADA (Legacy Input).");
                ToggleMenu();
            }
        }
        catch {} // Ignore if Legacy is disabled
    }

    public void ToggleMenu()
    {
        isPaused = !isPaused;
        Debug.Log($"[PauseMenu] ToggleMenu -> Nuevo estado isPaused: {isPaused}");
        
        // Use main thread dispatcher if needed, but performed info is on main thread usually
        if (menuCanvas != null)
        {
            menuCanvas.SetActive(isPaused);
        }

        // Unlock cursor so user can click buttons
        Cursor.visible = isPaused;
        Cursor.lockState = isPaused ? CursorLockMode.None : CursorLockMode.Locked;
    }

    public void GoToMainMenu()
    {
        Debug.Log("[PauseMenu] Loading Main Menu...");
        
        // Safely disconnect from HLA
        // We re-enable Disconnect() because we fixed the C++ crash issue.
        // This ensures the federate is resigned and frees the Pitch license slot.
        if (HlaNetworkManager.Instance != null && HlaNetworkManager.Instance.IsConnected)
        {
            Debug.Log("[PauseMenu] Disconnecting from HLA federation...");
            try 
            {
                // Disable time management first
                HlaInterface.DisableTimeManagement();
                HlaInterface.EvokeCallbacks(0.05);
                
                // NOW we can safely call Disconnect because we fixed the C++ plugin
                HlaInterface.Disconnect();
                Debug.Log("[PauseMenu] HLA Disconnect successful.");
            }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[PauseMenu] HLA cleanup warning: {e.Message}");
            }
            
            // Destroy the manager
            Destroy(HlaNetworkManager.Instance.gameObject);
        }

        SceneManager.LoadScene(mainMenuSceneName);
    }

    public void QuitGame()
    {
        Debug.Log("[PauseMenu] Quitting Application...");
        
        // This triggers OnApplicationQuit which sends our HLA disconnect signal (-9999)
        Application.Quit();

        // If in editor, stop playing
        #if UNITY_EDITOR
        UnityEditor.EditorApplication.isPlaying = false;
        #endif
    }
}
