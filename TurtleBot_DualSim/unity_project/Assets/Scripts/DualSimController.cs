using UnityEngine;
using HLA;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

/// <summary>
/// Controlador principal del DualSim en Unity.
/// 
/// - Envía la pose del robot local (el TurtleBot de Unity) a HLA.
/// - Recibe la pose del robot remoto (el TurtleBot de Gazebo) y 
///   crea/actualiza un ghost (dummy) para visualizarlo en la escena.
/// 
/// CONFIGURACIÓN:
/// - Adjuntar al GameObject del TurtleBot local de Unity.
/// - Asignar un prefab para el ghost en "ghostPrefab".
/// - Asegurar que DualSimNetworkManager está en la escena.
/// </summary>
public class DualSimController : MonoBehaviour
{
    [Header("Ghost Settings")]
    [Tooltip("Prefab para representar el robot remoto (ghost/dummy)")]
    public GameObject ghostPrefab;

    [Tooltip("Material semitransparente para el ghost (opcional)")]
    public Material ghostMaterial;

    [Header("Debug")]
    [Tooltip("Mostrar info de HLA en GUI")]
    public bool showDebugGUI = true;

    // Datos HLA
    private BoxData myBoxData;
    private bool hasCreatedUnit = false;

    // Ghost tracking
    private List<BoxData> remoteBoxes = new List<BoxData>();
    private Dictionary<int, GameObject> ghostObjects = new Dictionary<int, GameObject>();

    // Señal de desconexión
    private const float DISCONNECT_SIGNAL = -9999.0f;

    void Update()
    {
        if (DualSimNetworkManager.Instance == null ||
            !DualSimNetworkManager.Instance.IsConnected) return;

        // 1. Crear unidad HLA si no existe
        if (!hasCreatedUnit)
        {
            CreateMyUnit();
            return;
        }

        // 2. Esperar avance de tiempo HLA
        if (!DualSimNetworkManager.Instance.RequestAndWaitTimeAdvance(Time.deltaTime))
        {
            return; // Tiempo no concedido, saltar frame
        }

        // 3. Enviar pose local a HLA
        SendLocalPose();

        // 4. Recibir poses remotas desde HLA
        ReceiveRemotePoses();

        // 5. Actualizar ghosts 3D
        SyncGhosts();
    }

    void CreateMyUnit()
    {
        try
        {
            int id = HlaInterface.CreateUnit();
            myBoxData = new BoxData();
            myBoxData.id = id;
            hasCreatedUnit = true;
            Debug.Log($"[DualSim] Unidad local creada con ID: {id}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[DualSim] Error creando unidad: {e.Message}");
        }
    }

    void SendLocalPose()
    {
        // Unity: X=derecha, Y=arriba, Z=adelante
        // Gazebo: X=adelante, Y=izquierda, Z=arriba
        myBoxData.positionX = transform.position.z;  // X Gazebo (Adelante) = Z Unity
        myBoxData.positionY = -transform.position.x; // Y Gazebo (Izquierda) = -X Unity (Derecha)
        myBoxData.positionZ = transform.position.y;  // Z Gazebo (Arriba) = Y Unity
        
        // Rotación: Unity gira Y positivo en sentido horario. Gazebo gira Z positivo en antihorario.
        myBoxData.rotationY = -transform.eulerAngles.y;

        try
        {
            HlaInterface.UpdateUnit(myBoxData);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[DualSim] Error enviando pose: {e.Message}");
        }
    }

    void ReceiveRemotePoses()
    {
        try
        {
            int count = 0;
            IntPtr ptr = HlaInterface.GetBoxes(out count);

            remoteBoxes.Clear();

            // Debug periódico (cada ~60 frames ≈ 1s)
            if (Time.frameCount % 60 == 0)
            {
                Debug.Log($"[DualSim] GetBoxes: count={count}, ptr={(ptr != IntPtr.Zero ? "valid" : "null")}, myId={myBoxData.id}");
            }

            if (ptr != IntPtr.Zero && count > 0)
            {
                int structSize = Marshal.SizeOf(typeof(BoxData));
                for (int i = 0; i < count; i++)
                {
                    IntPtr currentPtr = new IntPtr(ptr.ToInt64() + (i * structSize));
                    BoxData box = (BoxData)Marshal.PtrToStructure(currentPtr, typeof(BoxData));

                    // Debug cada box recibido (periódico)
                    if (Time.frameCount % 60 == 0)
                    {
                        Debug.Log($"[DualSim]   Box[{i}]: id={box.id}, pos=({box.positionX:F2},{box.positionY:F2},{box.positionZ:F2}), rot={box.rotationY:F1}");
                    }

                    // Filtrar: no incluir nuestro propio robot
                    if (box.id == myBoxData.id) continue;

                    // Filtrar señal de desconexión
                    if (box.positionX <= -9990.0f) continue;

                    remoteBoxes.Add(box);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"[DualSim] Error recibiendo poses remotas: {e.Message}");
        }
    }

    void SyncGhosts()
    {
        HashSet<int> activeIds = new HashSet<int>();

        foreach (var data in remoteBoxes)
        {
            activeIds.Add(data.id);

            if (!ghostObjects.ContainsKey(data.id))
            {
                // Crear wrapper para rotación limpia
                GameObject wrapper = new GameObject($"Ghost_Gazebo_{data.id}");

                // Instanciar modelo como hijo
                GameObject model;
                if (ghostPrefab != null)
                {
                    model = Instantiate(ghostPrefab, wrapper.transform);
                }
                else
                {
                    // Fallback: cilindro como representación simple del robot
                    model = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                    model.transform.SetParent(wrapper.transform);
                    model.transform.localScale = new Vector3(0.3f, 0.1f, 0.3f);
                }

                model.transform.localPosition = Vector3.zero;

                // Aplicar material ghost si disponible
                if (ghostMaterial != null)
                {
                    ApplyMaterialRecursive(model, ghostMaterial);
                }
                else
                {
                    // Material semitransparente por defecto
                    ApplyDefaultGhostMaterial(model);
                }

                ghostObjects.Add(data.id, wrapper);
                Debug.Log($"[DualSim] Ghost creado para robot remoto ID: {data.id}");
            }

            ghostObjects[data.id].transform.position = new Vector3(
                -data.positionY,          // Unity X (Derecha) = -Gazebo Y (Izquierda)
                data.positionZ + 1.5f,    // Unity Y (Arriba) = Gazebo Z + 1.5m (suelo GZ a 0, suelo Unity a ~1.5)
                data.positionX            // Unity Z (Adelante) = Gazebo X (Adelante)
            );
            ghostObjects[data.id].transform.rotation = Quaternion.Euler(0f, -data.rotationY, 0f);
            
            // Log una vez para verificar coordenadas
            if (Time.frameCount % 120 == 0)
            {
                Debug.Log($"[DualSim] Ghost pos: HLA({data.positionX:F2},{data.positionY:F2},{data.positionZ:F2}) -> Unity({ghostObjects[data.id].transform.position})");
            }
        }

        // Eliminar ghosts de robots que ya no están
        List<int> toRemove = new List<int>();
        foreach (var id in ghostObjects.Keys)
        {
            if (!activeIds.Contains(id))
            {
                Destroy(ghostObjects[id]);
                toRemove.Add(id);
                Debug.Log($"[DualSim] Ghost eliminado para ID: {id}");
            }
        }
        foreach (var id in toRemove)
        {
            ghostObjects.Remove(id);
        }
    }

    void ApplyMaterialRecursive(GameObject obj, Material mat)
    {
        var renderer = obj.GetComponent<Renderer>();
        if (renderer != null) renderer.material = mat;

        foreach (Transform child in obj.transform)
        {
            ApplyMaterialRecursive(child.gameObject, mat);
        }
    }

    void ApplyDefaultGhostMaterial(GameObject obj)
    {
        var renderer = obj.GetComponent<Renderer>();
        if (renderer != null)
        {
            // Color azul semitransparente para distinguir el ghost
            Material ghostMat = new Material(Shader.Find("Standard"));
            ghostMat.SetFloat("_Mode", 3); // Transparent
            ghostMat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            ghostMat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            ghostMat.SetInt("_ZWrite", 0);
            ghostMat.DisableKeyword("_ALPHATEST_ON");
            ghostMat.EnableKeyword("_ALPHABLEND_ON");
            ghostMat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            ghostMat.renderQueue = 3000;
            ghostMat.color = new Color(0.2f, 0.5f, 1.0f, 0.5f);
            renderer.material = ghostMat;
        }
    }

    void OnApplicationQuit()
    {
        SendDisconnectSignal();
    }

    void OnDestroy()
    {
        SendDisconnectSignal();
    }

    void SendDisconnectSignal()
    {
        if (!hasCreatedUnit) return;

        Debug.Log("[DualSim] Enviando señal de desconexión...");
        myBoxData.positionX = DISCONNECT_SIGNAL;
        myBoxData.positionY = DISCONNECT_SIGNAL;
        myBoxData.rotationY = 0f;
        myBoxData.positionZ = 0f;

        try
        {
            HlaInterface.UpdateUnit(myBoxData);
        }
        catch (System.Exception) { }

        hasCreatedUnit = false;
    }

    void OnGUI()
    {
        if (!showDebugGUI) return;
        if (DualSimNetworkManager.Instance == null ||
            !DualSimNetworkManager.Instance.IsConnected) return;

        GUI.color = Color.white;

        // Info del robot local
        if (hasCreatedUnit)
        {
            GUI.Label(new Rect(10, 10, 500, 20),
                $"[DualSim] Robot Local (Unity) ID: {myBoxData.id}");
            GUI.Label(new Rect(10, 30, 500, 20),
                $"  Pos: ({myBoxData.positionX:F2}, {myBoxData.positionY:F2}, {myBoxData.positionZ:F2}) | Rot: {myBoxData.rotationY:F1}°");
        }

        // Info de robots remotos (ghosts)
        if (remoteBoxes.Count > 0)
        {
            GUI.Label(new Rect(10, 55, 300, 20),
                $"Robots remotos (Gazebo): {remoteBoxes.Count}");
            float y = 75;
            foreach (var box in remoteBoxes)
            {
                GUI.color = new Color(0.5f, 0.8f, 1.0f);
                GUI.Label(new Rect(10, y, 600, 20),
                    $"  Ghost ID: {box.id} | Pos: ({box.positionX:F2}, {box.positionY:F2}, {box.positionZ:F2}) | Rot: {box.rotationY:F1}°");
                y += 20;
            }
        }
        else
        {
            GUI.color = Color.yellow;
            GUI.Label(new Rect(10, 55, 300, 20),
                "Esperando robots remotos de Gazebo...");
        }
    }
}
