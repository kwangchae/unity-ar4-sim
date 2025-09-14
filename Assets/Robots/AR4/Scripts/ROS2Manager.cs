using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class ROS2Manager : MonoBehaviour
{
    [Header("ROS Connection")]
    [Tooltip("ROS TCP Endpoint IP address (WSL2 IP for Windows-WSL2 setup)")]
    public string rosIPAddress = "172.27.145.180";  // Your WSL2 IP
    [Tooltip("ROS TCP Endpoint port")]
    public int rosPort = 10000;
    [Tooltip("How often to publish joint states (Hz)")]
    public float publishRate = 30f;

    [Header("Robot Configuration")]
    [Tooltip("Name of the robot (used in frame_id)")]
    public string robotName = "ar4";
    [Tooltip("Joint names in order")]
    public string[] jointNames = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    [Header("References")]
    [Tooltip("AR4 Jogger Panel to get joint data from")]
    public AR4JoggerPanel joggerPanel;

    // ROS Topics
    const string JOINT_STATES_TOPIC = "/joint_states";
    const string JOINT_COMMAND_TOPIC = "/joint_command";

    // Internal state
    ROSConnection _rosConnection;
    float _lastPublishTime;
    JointStateMsg _jointStateMsg;
    bool _isConnected = false;

    void Start()
    {
        InitializeROS();
        SetupJointStateMessage();
    }

    void InitializeROS()
    {
        try
        {
            Debug.Log($"ROS2Manager: Attempting to connect to ROS at {rosIPAddress}:{rosPort}");

            // Ensure a single global ROS connection exists and is configured before connect
            _rosConnection = ROSConnection.GetOrCreateInstance();
            _rosConnection.RosIPAddress = rosIPAddress;
            _rosConnection.RosPort = rosPort;
            _rosConnection.ConnectOnStart = false; // we'll connect explicitly

            // Explicitly connect so registrations are sent immediately
            _rosConnection.Connect();

            Debug.Log($"ROS2Manager: ROS settings configured - IP: {_rosConnection.RosIPAddress}, Port: {_rosConnection.RosPort}");

            // Register publishers
            _rosConnection.RegisterPublisher<JointStateMsg>(JOINT_STATES_TOPIC);
            Debug.Log($"ROS2Manager: Registered publisher for {JOINT_STATES_TOPIC}");

            // Register subscribers
            _rosConnection.Subscribe<JointStateMsg>(JOINT_COMMAND_TOPIC, OnJointCommandReceived);
            Debug.Log($"ROS2Manager: Subscribed to {JOINT_COMMAND_TOPIC}");

            _isConnected = true;
            Debug.Log($"ROS2Manager: ROS initialization completed");
        }
        catch (Exception e)
        {
            Debug.LogError($"ROS2Manager: Failed to initialize ROS - {e.Message}");
            _isConnected = false;
        }
    }

    void SetupJointStateMessage()
    {
        _jointStateMsg = new JointStateMsg
        {
            header = new HeaderMsg(),
            name = jointNames,
            position = new double[jointNames.Length],
            velocity = new double[jointNames.Length],
            effort = new double[jointNames.Length]
        };
    }

    void Update()
    {
        if (!_isConnected || joggerPanel == null) return;

        // Publish joint states at specified rate
        if (Time.time - _lastPublishTime >= 1f / publishRate)
        {
            PublishJointStates();
            _lastPublishTime = Time.time;
        }
    }

    void PublishJointStates()
    {
        if (joggerPanel.joints == null || joggerPanel.joints.Length == 0) return;

        // Update header
        _jointStateMsg.header.stamp = new TimeMsg
        {
            sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
            nanosec = (uint)((DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000) * 1000000)
        };
        _jointStateMsg.header.frame_id = robotName;

        // Update joint data
        for (int i = 0; i < Math.Min(jointNames.Length, joggerPanel.joints.Length); i++)
        {
            var joint = joggerPanel.joints[i].joint;
            if (joint != null)
            {
                // Convert degrees to radians for ROS
                _jointStateMsg.position[i] = joint.xDrive.target * Mathf.Deg2Rad;
                
                // Calculate velocity (simple derivative)
                // TODO: Implement proper velocity calculation
                _jointStateMsg.velocity[i] = 0.0;
                
                // Effort/torque (not available from ArticulationBody directly)
                _jointStateMsg.effort[i] = 0.0;
            }
        }

        // Publish to ROS
        try
        {
            _rosConnection.Publish(JOINT_STATES_TOPIC, _jointStateMsg);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"ROS2Manager: Failed to publish joint states - {e.Message}");
        }
    }

    void OnJointCommandReceived(JointStateMsg commandMsg)
    {
        if (joggerPanel == null || joggerPanel.joints == null) return;

        Debug.Log($"ROS2Manager: Received joint command from ROS2 with {commandMsg.position.Length} positions");

        // Apply received joint positions to the robot
        for (int i = 0; i < Math.Min(commandMsg.position.Length, joggerPanel.joints.Length); i++)
        {
            var joint = joggerPanel.joints[i].joint;
            if (joint != null)
            {
                // Convert radians to degrees for Unity
                float targetDegrees = (float)(commandMsg.position[i] * Mathf.Rad2Deg);
                
                Debug.Log($"ROS2Manager: Setting joint {i} to {targetDegrees:F1} degrees");
                
                // Set the target through the jogger panel's SetDesiredAngle method
                joggerPanel.SetDesiredAngle(i, targetDegrees);
                
                // Also directly set the ArticulationBody target
                var drive = joint.xDrive;
                drive.target = targetDegrees;
                joint.xDrive = drive;
            }
        }
    }

    void OnApplicationQuit()
    {
        if (_isConnected && _rosConnection != null)
        {
            _rosConnection.Disconnect();
            Debug.Log("ROS2Manager: Disconnected from ROS");
        }
    }

    void OnGUI()
    {
        // Simple status display
        GUI.color = _isConnected ? Color.green : Color.red;
        GUI.Label(new Rect(10, 10, 200, 20), $"ROS2: {(_isConnected ? "Connected" : "Disconnected")}");
        GUI.color = Color.white;
        
        GUI.Label(new Rect(10, 30, 300, 20), $"Target IP: {rosIPAddress}:{rosPort}");
        
        if (_isConnected)
        {
            GUI.Label(new Rect(10, 50, 300, 20), $"Publishing to: {JOINT_STATES_TOPIC}");
            GUI.Label(new Rect(10, 70, 300, 20), $"Listening on: {JOINT_COMMAND_TOPIC}");
        }
        else
        {
            GUI.Label(new Rect(10, 50, 300, 20), "Waiting for connection...");
        }
    }
}
