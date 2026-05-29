using UnityEngine;

public class DayNightCycle : MonoBehaviour
{
    [Tooltip("How many real-time seconds it takes for a full day to pass")]
    public float secondsPerDay = 120f;

    void Update()
    {
        if (secondsPerDay <= 0f) return;

        // 360 degrees over the length of a day
        float degreesPerSecond = 360f / secondsPerDay;

        // Rotate around the X axis (sunrise/sunset motion)
        transform.Rotate(Vector3.right, degreesPerSecond * Time.deltaTime);
    }
}