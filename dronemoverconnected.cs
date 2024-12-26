using UnityEngine;
using System.Text;
using RJCP.IO.Ports;
using System.Threading;

public class DroneControllerSerial : MonoBehaviour
{
    public GameObject drone;
    private Vector3 position;
    private Vector3 velocity;
    private Vector3 acceleration;
    private Vector3 gyro;

    private SerialPortStream serialPort;
    private Thread readThread;
    private bool isReading = true;

    public string portName = "/dev/tty.usbmodem203E345C57461";
    public int baudRate = 115200;

    private float scaleFactor = 0.1f; // Scale of Unity world units

    void Start()
    {
        position = drone.transform.position;
        velocity = Vector3.zero;

        serialPort = new SerialPortStream(portName, baudRate);
        serialPort.DataBits = 8;
        serialPort.StopBits = StopBits.One;
        serialPort.Parity = Parity.None;
        serialPort.Open();

        if (serialPort.IsOpen)
        {
            Debug.Log($"Serial Port {portName} opened at {baudRate} baud.");
        }

        readThread = new Thread(ReadSerialData);
        readThread.IsBackground = true;
        readThread.Start();
    }

    void Update()
    {
        // 2x integrate acceleration to update velocity
        velocity += acceleration * Time.deltaTime;

        position += velocity * Time.deltaTime;

        // Apply the updated position to the drone
        drone.transform.position = position;

        Debug.Log($"Position: {position}, Velocity: {velocity}, Acceleration: {acceleration}, Gyro: {gyro}");
    }

    private void ReadSerialData()
    {
        while (isReading)
        {
            try
            {
                if (serialPort.IsOpen && serialPort.BytesToRead > 0)
                {
                    string line = serialPort.ReadLine();
                    ParseSerialData(line);
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"Error reading serial data: {ex.Message}");
            }
        }
    }

    private void ParseSerialData(string line)
    {
        string[] fields = line.Split(',');
        if (fields.Length >= 6)
        {
            try
            {
                acceleration = new Vector3(
                    float.Parse(fields[0]), // acc_x
                    float.Parse(fields[1]), // acc_y
                    float.Parse(fields[2])  // acc_z
                ) * scaleFactor;

                // Parse gyroscope data
                gyro = new Vector3(
                    float.Parse(fields[3]), // gyro_x
                    float.Parse(fields[4]), // gyro_y
                    float.Parse(fields[5])  // gyro_z
                );
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"Error parsing serial data: {ex.Message}");
            }
        }
    }

    private void OnDestroy()
    {
        isReading = false;

        if (serialPort != null && serialPort.IsOpen)
        {
            serialPort.Close();
        }

        if (readThread != null)
        {
            readThread.Abort();
        }
    }
}
