using UnityEngine;
using System;
using HLA;
using System.IO;
using System.Runtime.InteropServices;
using System.Collections.Generic;

public class HlaCoordinateViewer : MonoBehaviour
{
    [Header("HLA Settings")]
    public string federationName = "Test_Federation";
    public string federateName = "UnityViewer";
    public string fomFileName = "BoxFOM.xml";

    [Header("3D Visualization")]
    public GameObject playerPrefab;
    
    private List<BoxData> receivedBoxes = new List<BoxData>();
    private Dictionary<int, GameObject> spawnedObjects = new Dictionary<int, GameObject>();

    void Start()
    {
        
    }

    void Update()
    {
        if (HlaNetworkManager.Instance == null || !HlaNetworkManager.Instance.IsConnected) return;

        // === TIME MANAGEMENT CHECK ===
        // Only proceed with updates if time advance is granted
        if (!HlaNetworkManager.Instance.RequestAndWaitTimeAdvance(Time.deltaTime))
        {
            // Time advance not yet granted, skip this frame
            return;
        }

        UpdateBoxes();
        Sync3DModels();
    }

    void UpdateBoxes()
    {
        try
        {
            int count = 0;
            IntPtr ptr = HlaInterface.GetBoxes(out count);
            
            receivedBoxes.Clear();
            
            if (ptr != IntPtr.Zero && count > 0)
            {
                int structSize = Marshal.SizeOf(typeof(BoxData));
                for (int i = 0; i < count; i++)
                {
                    IntPtr currentPtr = new IntPtr(ptr.ToInt64() + (i * structSize));
                    BoxData box = (BoxData)Marshal.PtrToStructure(currentPtr, typeof(BoxData));
                    
                    // CHECK DISCONNECT SIGNAL
                    if (box.positionX <= -9990.0f)
                    {
                        continue;
                    }

                    receivedBoxes.Add(box);
                }
            }
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error fetching boxes: {e.Message}");
        }
    }

    void Sync3DModels()
    {
        HashSet<int> activeIds = new HashSet<int>();

        foreach (var data in receivedBoxes)
        {
            activeIds.Add(data.id);

            if (!spawnedObjects.ContainsKey(data.id))
            {
                // Create wrapper parent for clean HLA yaw rotation (avoids gimbal lock)
                GameObject wrapper = new GameObject($"HLA_Object_{data.id}");

                // Instantiate model as child (preserving prefab's internal mesh rotations)
                GameObject model = playerPrefab != null 
                    ? Instantiate(playerPrefab, wrapper.transform) 
                    : GameObject.CreatePrimitive(PrimitiveType.Cube);
                
                if (playerPrefab == null)
                    model.transform.SetParent(wrapper.transform);

                model.transform.localPosition = Vector3.zero;
                // Do NOT override localRotation — the prefab's child mesh has
                // baked import rotation that must be preserved

                spawnedObjects.Add(data.id, wrapper);
                Debug.Log($"[HLA Viewer] Spawned 3D model for Unit ID: {data.id}");
            }

            // Sync Position and Rotation (clean Y-only on wrapper, no gimbal lock)
            spawnedObjects[data.id].transform.position = new Vector3(data.positionX, data.positionZ, data.positionY);
            spawnedObjects[data.id].transform.rotation = Quaternion.Euler(0f, data.rotationY, 0f);
        }

        // Cleanup removed objects
        List<int> toRemove = new List<int>();
        foreach (var id in spawnedObjects.Keys)
        {
            if (!activeIds.Contains(id))
            {
                Destroy(spawnedObjects[id]);
                toRemove.Add(id);
                Debug.Log($"[HLA Viewer] Removed 3D model for Unit ID: {id}");
            }
        }

        foreach (var id in toRemove) spawnedObjects.Remove(id);
    }

    void OnGUI()
    {
        GUI.color = Color.black;

        if (HlaNetworkManager.Instance == null || !HlaNetworkManager.Instance.IsConnected)
        {
            GUI.Label(new Rect(10, 10, 300, 20), "HLA Viewer: Not Connected");
            return;
        }

        GUI.Label(new Rect(10, 10, 300, 20), $"HLA Viewer: Connected ({receivedBoxes.Count} active)");

        float y = 40;
        foreach (var box in receivedBoxes)
        {
            GUI.Label(new Rect(10, y, 600, 20), $"ID: {box.id} | X: {box.positionX:F2} | Y: {box.positionY:F2} | Z: {box.positionZ:F2} | Rot: {box.rotationY:F1}\u00b0");
            y += 20;
        }
    }

    // OnDestroy handled by Manager
    void OnDestroy() { }
}
