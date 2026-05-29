using UnityEngine;
using RosSharp.RosBridgeClient;
using Twist = RosSharp.RosBridgeClient.MessageTypes.Geometry.Twist;

public class Cmd_vel : UnitySubscriber<Twist>
{
    [Tooltip("ROS topic to subscribe (default: /cmd_vel)")]
    public string TopicName = "/cmd_vel";

    [Tooltip("Scale for linear velocity")]
    public float linearScale = 1.0f;

    [Tooltip("Scale for angular velocity")]
    public float angularScale = 1.0f;

    [Tooltip("Map ROS axes to Unity: ROS x->Unity z, ROS y->Unity x")]
    public bool useRosToUnityMapping = true;

    [Tooltip("Invert angular direction if rotation sign is reversed")]
    public bool invertAngular = false;

    private Vector3 linearVelocity = Vector3.zero;
    private float angularVelocityZ = 0f;
    private readonly object messageLock = new object();

    protected override void Start()
    {
        Topic = TopicName;
        base.Start();
    }

    protected override void ReceiveMessage(Twist message)
    {
        Vector3 lin;
        if (useRosToUnityMapping)
        {
            // ROS: x=forward, y=left, z=up
            // Unity: z=forward, x=right, y=up
            // Map ROS.x -> Unity.z, ROS.y -> Unity.x (no vertical movement)
            lin = new Vector3((float)message.linear.y * (-1), 0f, (float)message.linear.x * (-1));
        }
        else
        {
            lin = new Vector3((float)message.linear.x, (float)message.linear.y, (float)message.linear.z);
        }

        float angZ = (float)message.angular.z * (-1);

        lock (messageLock)
        {
            linearVelocity = lin;
            angularVelocityZ = angZ;
        }
    }

    void Update()
    {
        Vector3 lin;
        float ang;
        lock (messageLock)
        {
            lin = linearVelocity;
            ang = angularVelocityZ;
        }

        // Move in local space using the received linear velocity
        transform.Translate(lin * linearScale * Time.deltaTime, Space.Self);

        // Rotate around local Y (up). Twist angular values are in rad/s, convert to degrees/s
        float angSign = invertAngular ? -1f : 1f;
        transform.Rotate(0f, angSign * ang * angularScale * Mathf.Rad2Deg * Time.deltaTime, 0f);
    }
}
