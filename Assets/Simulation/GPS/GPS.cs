using System;
using System.Collections;
using System.Collections.Generic;

using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class GPS : MonoBehaviour
{
    public struct Coords
    {
        public double latitude;
        public double longitude;
        public float altitude;
    }

    [SerializeField]
    private string frameId = "gps_link";

    [SerializeField]
    private string topic = "/gps/fix";

    [SerializeField]
    private float frequency = 5;

    [SerializeField]
    private bool enableAltitude = false;

    [SerializeField]
    private float noiseStdDev = 1.0F;
    [SerializeField]
    private float driftUpdateRate = 0.2F;

    private Planet planet;
    private ROSConnection ros;
    private double lastPublishTime = 0.0;
    private GPSBaseStation[] baseStations;
    private Vector2 centerLatLon;
    private bool initialized = false;
    private Coords lastCoords;
    private bool hasFix = false;
    private bool lastCoordsWerePublished = false;
    private Vector3 targetDrift = Vector3.zero;
    private Vector3 currentDrift = Vector3.zero;
    private double lastDriftUpdateTime = 0.0;


    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>(topic);
        ScanBaseStations();
    }


    private void OnApplicationQuit()
    {
        // ROSConnectionはUnityのライフサイクルで自動管理されるため、明示的なCloseは不要
    }

    internal void ScanBaseStations()
    {
        baseStations = FindObjectsOfType<GPSBaseStation>();
        if (baseStations.Length < 1)
        {
            Debug.LogWarning("No GPS base stations found in the scene.");
            return;
        }
        else
        {
            Debug.Log($"Found {baseStations.Length} GPS base stations in the scene.");
        }

        // Initialize the planet with a default radius (e.g., Earth's radius in meters)
        planet = new Planet(6371000.0); // Earth's radius in meters

        // For simplicity, just use the first base station as the center
        centerLatLon = new Vector2((float)baseStations[0].latitude, (float)baseStations[0].longitude);
        initialized = true;
    }

    void FixedUpdate()
    {
        if (!initialized)
        {
            return;
        }

        // ここではダミーでUnity座標を緯度経度に変換（実際は正確な変換式を使うこと）
        // 例: X=経度, Z=緯度, Y=高度
        double latitude = centerLatLon.x + transform.position.z * 0.00001; // 仮の変換
        double longitude = centerLatLon.y + transform.position.x * 0.00001; // 仮の変換
        float altitude = enableAltitude ? (float)transform.position.y : 0f;

        lastCoords = new Coords
        {
            latitude = latitude,
            longitude = longitude,
            altitude = altitude
        };
        hasFix = true;
        lastCoordsWerePublished = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (!hasFix || lastCoordsWerePublished)
            return;

        double now = Time.realtimeSinceStartupAsDouble;
        if (now - lastPublishTime < 1.0 / frequency)
            return;
        lastPublishTime = now;

        double variance = noiseStdDev * noiseStdDev;
        var msg = new NavSatFixMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg { sec = (int)now, nanosec = (uint)((now - (int)now) * 1e9) }
            },
            status = new NavSatStatusMsg
            {
                status = hasFix ? NavSatStatusMsg.STATUS_FIX : NavSatStatusMsg.STATUS_NO_FIX,
                service = NavSatStatusMsg.SERVICE_GPS
            },
            latitude = lastCoords.latitude,
            longitude = lastCoords.longitude,
            altitude = enableAltitude ? lastCoords.altitude : double.NaN,
            position_covariance = new double[9] { variance, 0, 0, 0, variance, 0, 0, 0, variance },
            position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_APPROXIMATED
        };
        ros.Publish(topic, msg);
        lastCoordsWerePublished = true;
    }
}
