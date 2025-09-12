using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class TrajectoryVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    [Tooltip("Prefab for waypoint visualization")]
    public GameObject waypointPrefab;
    
    [Tooltip("Line material for trajectory path")]
    public Material trajectoryLineMaterial;
    
    [Tooltip("Colors for different trajectory stages")]
    public Color plannedColor = Color.yellow;
    public Color executingColor = Color.green;
    public Color completedColor = Color.blue;
    
    [Header("Settings")]
    public float waypointScale = 0.05f;
    public float lineWidth = 0.01f;
    public bool showWaypoints = true;
    public bool showTrajectoryLine = true;
    
    // Internal state
    private List<GameObject> waypointObjects = new List<GameObject>();
    private LineRenderer trajectoryLine;
    private List<Vector3> waypointPositions = new List<Vector3>();
    
    // ROS connection
    private ROSConnection rosConnection;
    
    void Start()
    {
        // Initialize ROS connection
        rosConnection = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to trajectory preview topic
        rosConnection.Subscribe<JointStateMsg>("/trajectory_preview", OnTrajectoryWaypointReceived);
        
        // Setup trajectory line renderer
        SetupTrajectoryLine();
        
        // Create default waypoint prefab if not assigned
        if (waypointPrefab == null)
        {
            CreateDefaultWaypointPrefab();
        }
        
        Debug.Log("TrajectoryVisualizer: Initialized and subscribed to /trajectory_preview");
    }
    
    void SetupTrajectoryLine()
    {
        // Create line renderer for trajectory visualization
        GameObject lineObj = new GameObject("TrajectoryLine");
        lineObj.transform.parent = transform;
        
        trajectoryLine = lineObj.AddComponent<LineRenderer>();
        trajectoryLine.material = trajectoryLineMaterial;
        trajectoryLine.startWidth = lineWidth;
        trajectoryLine.endWidth = lineWidth;
        trajectoryLine.positionCount = 0;
        trajectoryLine.useWorldSpace = true;
        
        // Create default material if not assigned
        if (trajectoryLineMaterial == null)
        {
            trajectoryLineMaterial = new Material(Shader.Find("Sprites/Default"));
            trajectoryLineMaterial.color = plannedColor;
            trajectoryLine.material = trajectoryLineMaterial;
        }
    }
    
    void CreateDefaultWaypointPrefab()
    {
        // Create a simple sphere as default waypoint marker
        waypointPrefab = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        waypointPrefab.name = "WaypointMarker";
        
        // Remove collider to avoid physics interference
        DestroyImmediate(waypointPrefab.GetComponent<Collider>());
        
        // Make it a prefab (disable in hierarchy)
        waypointPrefab.SetActive(false);
    }
    
    void OnTrajectoryWaypointReceived(JointStateMsg jointState)
    {
        // Check if this is the start of a new trajectory
        if (jointState.header.frame_id.Contains("waypoint_0"))
        {
            ClearPreviousTrajectory();
            waypointPositions.Clear();
        }
        
        // Calculate end-effector position from joint state
        Vector3 eePosition = CalculateEndEffectorPosition(jointState.position);
        waypointPositions.Add(eePosition);
        
        // Create waypoint marker
        if (showWaypoints)
        {
            CreateWaypointMarker(eePosition, waypointPositions.Count - 1);
        }
        
        // Update trajectory line
        if (showTrajectoryLine)
        {
            UpdateTrajectoryLine();
        }
        
        Debug.Log($"TrajectoryVisualizer: Added waypoint {waypointPositions.Count} at {eePosition}");
    }
    
    Vector3 CalculateEndEffectorPosition(double[] jointPositions)
    {
        // Simple forward kinematics calculation for AR4
        // This is a simplified version - in practice, you'd want more accurate FK
        
        if (jointPositions.Length < 6) return Vector3.zero;
        
        // AR4 approximate link lengths (in meters)
        float L1 = 0.169f;  // Base to joint 2
        float L2 = 0.305f;  // Upper arm length
        float L3 = 0.305f;  // Forearm length
        float L4 = 0.07f;   // Wrist to end-effector
        
        // Convert to Unity coordinate system and scale
        float j1 = (float)jointPositions[0];  // Base rotation
        float j2 = (float)jointPositions[1];  // Shoulder
        float j3 = (float)jointPositions[2];  // Elbow
        float j5 = (float)jointPositions[4];  // Wrist pitch
        
        // Simplified forward kinematics
        float x = Mathf.Cos(j1) * (L2 * Mathf.Cos(j2) + L3 * Mathf.Cos(j2 + j3) + L4 * Mathf.Cos(j2 + j3 + j5));
        float y = Mathf.Sin(j1) * (L2 * Mathf.Cos(j2) + L3 * Mathf.Cos(j2 + j3) + L4 * Mathf.Cos(j2 + j3 + j5));
        float z = L1 + L2 * Mathf.Sin(j2) + L3 * Mathf.Sin(j2 + j3) + L4 * Mathf.Sin(j2 + j3 + j5);
        
        return new Vector3(x, z, y); // Unity coordinate conversion
    }
    
    void CreateWaypointMarker(Vector3 position, int waypointIndex)
    {
        GameObject waypoint = Instantiate(waypointPrefab, position, Quaternion.identity, transform);
        waypoint.SetActive(true);
        waypoint.name = $"Waypoint_{waypointIndex}";
        waypoint.transform.localScale = Vector3.one * waypointScale;
        
        // Color coding based on position in trajectory
        Renderer renderer = waypoint.GetComponent<Renderer>();
        if (renderer != null)
        {
            Material mat = new Material(renderer.material);
            float t = waypointIndex / Mathf.Max(1f, waypointPositions.Count - 1);
            mat.color = Color.Lerp(plannedColor, executingColor, t);
            renderer.material = mat;
        }
        
        // Add text label
        GameObject label = new GameObject("Label");
        label.transform.parent = waypoint.transform;
        label.transform.localPosition = Vector3.up * 0.02f;
        
        TextMesh textMesh = label.AddComponent<TextMesh>();
        textMesh.text = waypointIndex.ToString();
        textMesh.fontSize = 20;
        textMesh.anchor = TextAnchor.MiddleCenter;
        textMesh.color = Color.white;
        textMesh.transform.localScale = Vector3.one * 0.01f;
        
        waypointObjects.Add(waypoint);
    }
    
    void UpdateTrajectoryLine()
    {
        if (trajectoryLine != null && waypointPositions.Count > 1)
        {
            trajectoryLine.positionCount = waypointPositions.Count;
            trajectoryLine.SetPositions(waypointPositions.ToArray());
        }
    }
    
    void ClearPreviousTrajectory()
    {
        // Clear waypoint objects
        foreach (GameObject waypoint in waypointObjects)
        {
            if (waypoint != null)
            {
                DestroyImmediate(waypoint);
            }
        }
        waypointObjects.Clear();
        
        // Clear trajectory line
        if (trajectoryLine != null)
        {
            trajectoryLine.positionCount = 0;
        }
        
        Debug.Log("TrajectoryVisualizer: Cleared previous trajectory");
    }
    
    [ContextMenu("Clear Trajectory")]
    public void ClearTrajectoryManual()
    {
        ClearPreviousTrajectory();
        waypointPositions.Clear();
    }
    
    void OnGUI()
    {
        // Simple status display
        GUILayout.BeginArea(new Rect(10, 100, 300, 100));
        GUILayout.Label($"Trajectory Waypoints: {waypointPositions.Count}", GUI.skin.box);
        
        GUI.color = showWaypoints ? Color.green : Color.red;
        if (GUILayout.Button($"Waypoints: {(showWaypoints ? "ON" : "OFF")}"))
        {
            showWaypoints = !showWaypoints;
            // Toggle existing waypoints
            foreach (GameObject waypoint in waypointObjects)
            {
                if (waypoint != null)
                {
                    waypoint.SetActive(showWaypoints);
                }
            }
        }
        
        GUI.color = showTrajectoryLine ? Color.green : Color.red;
        if (GUILayout.Button($"Trajectory Line: {(showTrajectoryLine ? "ON" : "OFF")}"))
        {
            showTrajectoryLine = !showTrajectoryLine;
            if (trajectoryLine != null)
            {
                trajectoryLine.enabled = showTrajectoryLine;
            }
        }
        
        GUI.color = Color.white;
        if (GUILayout.Button("Clear Trajectory"))
        {
            ClearTrajectoryManual();
        }
        
        GUILayout.EndArea();
    }
    
    void OnDestroy()
    {
        ClearPreviousTrajectory();
    }
}