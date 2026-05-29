using UnityEngine;
using HLA;
using System.IO;
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

public class HlaPlayerSender : MonoBehaviour
{
    [Header("HLA Settings")]
    public string federationName = "Test_Federation";
    public string federateName = "UnitySender";
    public string fomFileName = "BoxFOM.xml"; 

    [Header("3D Visualization")]
    public GameObject otherPlayerPrefab;

    private BoxData myBoxData;
    private bool isConnected = false;
    private bool hasCreatedUnit = false;

    // Visualization tracking
    private List<BoxData> remoteBoxes = new List<BoxData>();
    private Dictionary<int, GameObject> spawnedObjects = new Dictionary<int, GameObject>();
    
    // Explicit disconnect signal value
    private const float DISCONNECT_SIGNAL = -9999.0f;

    void Start()
    {
        // Wait for Manager
    }

    void Update()
    {
        if (HlaNetworkManager.Instance == null || !HlaNetworkManager.Instance.IsConnected) return;

        // 1. Handle Local Player Publication
        if (!hasCreatedUnit)
        {
            CreateMyUnit();
            return; 
        }

        // === TIME MANAGEMENT CHECK ===
        // Only proceed with simulation updates if time advance is granted
        if (!HlaNetworkManager.Instance.RequestAndWaitTimeAdvance(Time.deltaTime))
        {
            // Time advance not yet granted, skip this frame
            return;
        }

        UpdateLocalPlayer();

        // 2. Handle Remote Player Subscription (Multiplayer)
        UpdateRemoteBoxes();
        Sync3DModels();
    }

    void UpdateLocalPlayer()
    {
        myBoxData.positionX = transform.position.x;
        myBoxData.positionY = transform.position.z;
        myBoxData.rotationY = transform.eulerAngles.y;
        myBoxData.positionZ = transform.position.y;

        try
        {
            HlaInterface.UpdateUnit(myBoxData);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error updating HLA unit: {e.Message}");
        }
    }

    void UpdateRemoteBoxes()
    {
        try
        {
            int count = 0;
            IntPtr ptr = HlaInterface.GetBoxes(out count);

            receivedBoxesFromRTI(ptr, count);
        }
        catch (System.Exception e)
        {
            Debug.LogWarning($"Error fetching boxes: {e.Message}");
        }
    }
    
    void receivedBoxesFromRTI(IntPtr ptr, int count)
    {
        remoteBoxes.Clear();
        
        if (ptr != IntPtr.Zero && count > 0)
        {
            int structSize = Marshal.SizeOf(typeof(BoxData));
            for (int i = 0; i < count; i++)
            {
                IntPtr currentPtr = new IntPtr(ptr.ToInt64() + (i * structSize));
                BoxData box = (BoxData)Marshal.PtrToStructure(currentPtr, typeof(BoxData));
                
                // SELF-FILTER
                if (box.id == myBoxData.id) continue;

                // CHECK DISCONNECT SIGNAL
                if (box.positionX <= -9990.0f)
                {
                    continue; 
                }

                remoteBoxes.Add(box);
            }
        }
    }

    void Sync3DModels()
    {
        HashSet<int> activeIds = new HashSet<int>();

        foreach (var data in remoteBoxes)
        {
            activeIds.Add(data.id);

            if (!spawnedObjects.ContainsKey(data.id))
            {
                // Create wrapper parent for clean HLA yaw rotation (avoids gimbal lock)
                GameObject wrapper = new GameObject($"RemotePlayer_{data.id}");

                // Instantiate model as child (preserving prefab's internal mesh rotations)
                GameObject model = otherPlayerPrefab != null 
                    ? Instantiate(otherPlayerPrefab, wrapper.transform) 
                    : GameObject.CreatePrimitive(PrimitiveType.Cube);
                
                if (otherPlayerPrefab == null)
                    model.transform.SetParent(wrapper.transform);

                model.transform.localPosition = Vector3.zero;
                // Do NOT override localRotation — the prefab's child mesh has
                // baked import rotation (-90, 0, 90) that must be preserved

                spawnedObjects.Add(data.id, wrapper);
            }

            // Sync Position and Rotation (clean Y-only on wrapper, no gimbal lock)
            spawnedObjects[data.id].transform.position = new Vector3(data.positionX, data.positionZ, data.positionY);
            spawnedObjects[data.id].transform.rotation = Quaternion.Euler(0f, data.rotationY, 0f);
        }

        // Cleanup objects that are no longer in the list (or were filtered out because of -9999)
        List<int> toRemove = new List<int>();
        foreach (var id in spawnedObjects.Keys)
        {
            if (!activeIds.Contains(id))
            {
                Destroy(spawnedObjects[id]);
                toRemove.Add(id);
                Debug.Log($"[HLA Player] Removed Remote Player {id} (Disconnected/Signal)");
            }
        }

        foreach (var id in toRemove) 
        {
            spawnedObjects.Remove(id);
        }
    }

    void CreateMyUnit()
    {
        try
        {
            int id = HlaInterface.CreateUnit();
            myBoxData = new BoxData();
            myBoxData.id = id;
            hasCreatedUnit = true;
            Debug.Log($"Sender created Local Unit ID: {id}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error creating unit: {e.Message}");
        }
    }
    
    void OnApplicationQuit()
    {
        SendDisconnectSignal();
    }

    void OnGUI()
    {
        if (!isConnected || remoteBoxes.Count == 0) 
        {
            isConnected = HlaNetworkManager.Instance != null && HlaNetworkManager.Instance.IsConnected;
            if (!isConnected) return;
        }

        GUI.color = Color.black;
        GUI.Label(new Rect(10, 10, 500, 20), $"HLA Player: Connected (Local ID: {myBoxData.id})");
        GUI.Label(new Rect(10, 30, 500, 20), $"Local | X: {myBoxData.positionX:F2} | Y: {myBoxData.positionY:F2} | Z: {myBoxData.positionZ:F2} | Rot: {myBoxData.rotationY:F1}\u00b0");
        
        if (remoteBoxes.Count > 0)
        {
            GUI.Label(new Rect(10, 50, 300, 20), $"Visible remote players: {remoteBoxes.Count}");
            float y = 70;
            foreach (var box in remoteBoxes)
            {
                GUI.Label(new Rect(10, y, 600, 20), $"Remote ID {box.id} | X: {box.positionX:F2} | Y: {box.positionY:F2} | Z: {box.positionZ:F2} | Rot: {box.rotationY:F1}\u00b0");
                y += 20;
            }
        }
        else
        {
            GUI.Label(new Rect(10, 50, 300, 20), "No other federates detected...");
        }
    }

    void OnDestroy() 
    {
        SendDisconnectSignal();
    }

    private void SendDisconnectSignal()
    {
        if (hasCreatedUnit)
        {
            Debug.Log("[HLA Sender] Sending Disconnect Signal...");
            myBoxData.positionX = DISCONNECT_SIGNAL;
            myBoxData.positionY = DISCONNECT_SIGNAL;
            myBoxData.rotationY = 0f;
            myBoxData.positionZ = 0f;
            try
            {
                // Ensure Interface is still valid (might be destroyed if Manager goes first)
                HlaInterface.UpdateUnit(myBoxData);
            }
            catch (System.Exception e)
            {
                // Silent catch, standard during shutdown race conditions
            }
            
            // Prevent double-sending
            hasCreatedUnit = false; 
        }
    }
}
