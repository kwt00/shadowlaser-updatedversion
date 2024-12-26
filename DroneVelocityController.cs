using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using TMPro;
using System;


public class DroneVelocityController : MonoBehaviour
{
    [Header("Drone Settings")]
    public GameObject drone;

    [Header("UI Settings")]
    public TMP_Text gpsOverlayText;

    [Header("Raytracing Settings")]
    public int rayCount = 10;       // Number of rays to cast
    public float rayRange = 100f;   // Maximum range of each ray
    public LayerMask terrainLayer;  // Layer to detect terrain objects


    private Vector3 velocity = Vector3.zero; // Current velocity of drone
    private Vector3 positionOffset = Vector3.zero; // Position offset based on takeoff location
    private Vector3 pausedPosition = Vector3.zero;
    private bool isPaused = false; // Pause state

    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;

    private const int port = 51912; // Match UDP port in py script

    // Mapping constants for Unity -> GPS
    private const double LAT_REF = 37.778369;
    private const double LON_REF = -122.390004;
    private const double C = 3915.367;
    private const double F = 1287.471;
    private const double A = 0.13350;
    private const double B = -0.000166;
    private const double D = -0.000174;
    private const double E = 0.13450;
    private const double METERS_PER_DEG_LAT = 111000;
    private double METERS_PER_DEG_LON;

    public static DroneVelocityController Instance { get; private set; }

    // Broadcast GPS updates
    public event Action<double, double> OnGPSUpdate;

    // Current GPS coords
    private double currentLatitude;
    private double currentLongitude;

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
        udpClient = new UdpClient(port);
        remoteEndPoint = new IPEndPoint(IPAddress.Any, port);

        // Initialize METERS_PER_DEG_LON based on average latitude
        double AVG_LAT_RAD = 37.896032 * Mathf.Deg2Rad;
        METERS_PER_DEG_LON = 111000 * Math.Cos(AVG_LAT_RAD);

        positionOffset = Vector3.zero;

        Debug.Log($"Listening for UDP data on port {port}...");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            TogglePauseResume();
        }

        if (!isPaused)
        {
            // Wait for data
            if (udpClient.Available > 0)
            {
                try
                {
                    byte[] data = udpClient.Receive(ref remoteEndPoint);
                    string jsonData = Encoding.UTF8.GetString(data);
                    VelocityData velocityData = JsonUtility.FromJson<VelocityData>(jsonData);

                    if (velocityData != null)
                    {
                        velocity = new Vector3(
                            velocityData.velocity.x,
                            velocityData.velocity.y,
                            velocityData.velocity.z
                        );

                        // Update position offset if not already set
                        if (positionOffset == Vector3.zero)
                        {
                            positionOffset = new Vector3(
                                velocityData.position_offset.x,
                                0f,
                                velocityData.position_offset.z
                            );

                            // Set initial drone position
                            drone.transform.position = positionOffset;
                        }

                        // Use dt from the packet
                        float dt = velocityData.dt;

                        // Move drone based on velocity, dt
                        drone.transform.position += velocity * dt;

                        // Update GPS overlay
                        UpdateGPSOverlay(drone.transform.position, velocityData.timestamp);

                        // GPS update
                        OnGPSUpdate?.Invoke(currentLatitude, currentLongitude);
                    }
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"Error receiving UDP data: {ex.Message}");
                }
            }
        }
        else
        {
            // TODO: Implement any exploration controls here if needed
        }
    }

    private void UpdateGPSOverlay(Vector3 unityPosition, float timestamp)
    {
        // Unity position -> GPS coords
        double[] gpsCoords = UnityToGPS(unityPosition.x, unityPosition.z);

        // Update current GPS coords
        currentLatitude = gpsCoords[0];
        currentLongitude = gpsCoords[1];

        if (gpsOverlayText != null)
        {
            gpsOverlayText.text = $"Latitude: {gpsCoords[0]:F6}°\nLongitude: {gpsCoords[1]:F6}°";
        }
    }

    private double[] UnityToGPS(double xUnity, double zUnity)
    {
        // Solve the linear equations to get deltaE and deltaN
        // X_unity = A * deltaE + B * deltaN + C
        // Z_unity = D * deltaE + E * deltaN + F

        // Build matrices for linear equations
        double[,] matrix = {
            { A, B },
            { D, E }
        };
        double[] constants = {
            xUnity - C,
            zUnity - F
        };

        double deltaE, deltaN;
        SolveLinearEquations(matrix, constants, out deltaE, out deltaN);

        // Convert deltaE and deltaN to lat and long diffs
        double deltaLat = deltaN / METERS_PER_DEG_LAT;
        double deltaLon = deltaE / METERS_PER_DEG_LON;

        // Compute GPS coords
        double latitude = LAT_REF + deltaLat;
        double longitude = LON_REF + deltaLon;

        return new double[] { latitude, longitude };
    }

    private void SolveLinearEquations(double[,] matrix, double[] constants, out double x, out double y)
    {
        // Cramers Rule to solve 2x2 linear equations
        double a = matrix[0, 0];
        double b = matrix[0, 1];
        double c = matrix[1, 0];
        double d = matrix[1, 1];
        double e = constants[0];
        double f = constants[1];

        double denominator = a * d - b * c;
        if (denominator == 0)
        {
            x = 0;
            y = 0;
            Debug.LogError("Cannot solve linear equations: determinant is zero.");
            return;
        }

        x = (e * d - b * f) / denominator;
        y = (a * f - e * c) / denominator;
    }

    private void TogglePauseResume()
    {
        if (isPaused)
        {
            // Resume
            isPaused = false;
            // Snap back to paused position
            drone.transform.position = pausedPosition;
            Debug.Log("Resumed flight.");
        }
        else
        {
            // Pause
            isPaused = true;
            // Record paused position
            pausedPosition = drone.transform.position;
            Debug.Log("Paused flight.");
        }
    }

    private void OnDestroy()
    {
        udpClient.Close();
    }

    // Define VelocityData structure to match JSON data
    [System.Serializable]
    private class VelocityData
    {
        public float timestamp;
        public float dt;
        public Velocity velocity;
        public PositionOffset position_offset;

        [System.Serializable]
        public class Velocity
        {
            public float x, y, z;
        }

        [System.Serializable]
        public class PositionOffset
        {
            public float x, z;
        }
    }
    private void PerformSimulatedRaycasting()
    {
        Vector3 origin = drone.transform.position; // Raycasting starts from current drone position
    
        for (int i = 0; i < rayCount; i++)
        {
        
            Vector3 direction = drone.transform.forward;

            if (Physics.Raycast(origin, direction, out RaycastHit hit, rayRange, terrainLayer))
            {
                // Visualize ray in Scene view
                Debug.DrawLine(origin, hit.point, Color.red);
    
                // Get Unity position of hit point
                Vector3 hitPoint = hit.point;
    
                // Convert the hit point -> GPS coords
                double[] gpsCoords = UnityToGPS(hitPoint.x, hitPoint.z);
    
                Debug.Log($"Ray {i}: Hit at Unity position {hitPoint}, GPS Coordinates: Lat {gpsCoords[0]:F6}, Lon {gpsCoords[1]:F6}");
    
                OnGPSUpdate?.Invoke(gpsCoords[0], gpsCoords[1]);
            }
        }
    }
}
