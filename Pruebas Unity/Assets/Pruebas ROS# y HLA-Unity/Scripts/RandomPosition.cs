using UnityEngine;

public class RandomPosition : MonoBehaviour
{
    [Header("Spawn Area Settings")]
    [Tooltip("Minimum X coordinate")]
    public float minX = -10.0f;
    [Tooltip("Maximum X coordinate")]
    public float maxX = 10.0f;
    
    [Tooltip("Minimum Z coordinate")]
    public float minZ = -10.0f;
    [Tooltip("Maximum Z coordinate")]
    public float maxZ = 10.0f;

    [Tooltip("Optional: Fixed Y height (leave as 0 to keep current Y)")]
    public float fixedY = 0.51f;
    public bool useCurrentY = true;

    void Start()
    {
        float randomX = Random.Range(minX, maxX);
        float randomZ = Random.Range(minZ, maxZ);
        float yPos = useCurrentY ? transform.position.y : fixedY;

        transform.position = new Vector3(randomX, yPos, randomZ);
        
        Debug.Log($"[RandomPosition] Moving {gameObject.name} to: {transform.position}");
    }
}
