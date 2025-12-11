using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;

public class IMU : MonoBehaviour
{
    [SerializeField]
    private string frameId = "imu_link";
    [SerializeField]
    private string topic = "/imu/data";
    [SerializeField]
    private float frequency = 60;
    [SerializeField]
    private bool useMag = true;
    [SerializeField]
    private float magOffset = 0; // rad
    [SerializeField]
    private float gravity = 9.81f;

    private Vector3 prevPos;
    private Vector3 prevVel;
    private Vector3 prevAcc;
    private Quaternion prevAngPos;
    private Quaternion prevAngVel;
    private ROSConnection ros;
    private double lastPublishTime = 0.0;
    private double[] covariance = new double[9] { 1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3 };
    private float noMagYawError = 0.0f;

    [Header("Heading Publishing")]
    [SerializeField]
    private string headingTopic = "/imu/heading";
    [SerializeField]
    private double planetRadius = 6371000.0; // meters (Earth default)
    [SerializeField]
    private Vector2 referenceLatLon = new Vector2(0, 0); // degrees
    private Planet planet;

    private void Start()
    {
        prevPos = transform.position;
        prevVel = Vector3.zero;
        prevAngPos = Quaternion.identity;

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topic);
        ros.RegisterPublisher<Float64Msg>(headingTopic);

        planet = new Planet(planetRadius);

        FixedUpdate();
    }

    private void OnApplicationQuit()
    {
        // ROSConnectionはUnityのライフサイクルで自動管理されるため、明示的なCloseは不要
    }

    private void FixedUpdate()
    {
        // linear update
        Vector3 pos = transform.position;
        Vector3 vel = (pos - prevPos) / Time.fixedDeltaTime;
        Vector3 acc = (vel - prevVel) / Time.fixedDeltaTime;
        prevPos = pos;
        prevVel = vel;
        prevAcc = acc;

        // angular update
        Quaternion angPos = transform.rotation; // world space rotation as a quaternion
        Quaternion deltaAngPos = angPos * Quaternion.Inverse(prevAngPos);
        // Vector3 deltaAngPosAxis = new Vector3(deltaAngPos.x, deltaAngPos.y, deltaAngPos.z).normalized;
        // float deltaAngPosAngle = Mathf.Acos(deltaAngPos.w) * 2.0f;
        // Quaternion angVel = Quaternion.AngleAxis(deltaAngPosAngle / Time.fixedDeltaTime, deltaAngPosAxis);
        Quaternion angVel = Quaternion.SlerpUnclamped(Quaternion.identity, deltaAngPos, 1 / Time.fixedDeltaTime);
        prevAngPos = angPos;
        prevAngVel = angVel;

        // update noMagYawError
        if (!useMag)
        {
            float angSpeed = Quaternion.Angle(Quaternion.identity, angVel) * Mathf.Deg2Rad;
            noMagYawError += angSpeed * Time.fixedDeltaTime * UnityEngine.Random.Range(-0.05f, 0.3f);
        } else {
            noMagYawError = 0.0f;
        }
    }

    private void Update()
    {
        // publish to ROS2 using ROSTCPConnector
        double currentTime = Time.unscaledTimeAsDouble;
        if (currentTime - lastPublishTime < 1.0 / frequency)
        {
            return;
        }
        lastPublishTime = currentTime;

        Quaternion angPos = prevAngPos;
        Quaternion angVel = prevAngVel;
        Vector3 acc = prevAcc;
        acc.y += gravity; // gravity compensation
        acc = transform.InverseTransformDirection(acc); // make acceleration local to the sensor

        Matrix4x4 rosToUnity = Matrix4x4.identity;
        rosToUnity.SetColumn(0, new Vector4(0, 0, 1, 0));
        rosToUnity.SetColumn(1, new Vector4(-1, 0, 0, 0));
        rosToUnity.SetColumn(2, new Vector4(0, 1, 0, 0));
        Matrix4x4 unityToRos = rosToUnity.inverse;

        // Transform absolute orientation to ROS coordinate system.
        Matrix4x4 unityToOffsetUnity = Matrix4x4.Rotate(Quaternion.AngleAxis(Mathf.Rad2Deg * (magOffset + noMagYawError), Vector3.up));
        Matrix4x4 angPosMat = Matrix4x4.Rotate(angPos);
        angPosMat = unityToRos * unityToOffsetUnity * angPosMat * rosToUnity;
        angPos = angPosMat.rotation;

        // Transform angular velocity to ROS coordinate system.
        Matrix4x4 angVelMat = Matrix4x4.Rotate(angVel);
        angVelMat = unityToRos * angVelMat * rosToUnity;
        angVel = angVelMat.rotation;

        // Transform acceleration vector to ROS coordinate system.
        acc = unityToRos.MultiplyVector(acc);

        // Create and publish ImuMsg
        var msg = new ImuMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)currentTime,
                    nanosec = (uint)((currentTime - (int)currentTime) * 1e9)
                }
            },
            orientation = new RosMessageTypes.Geometry.QuaternionMsg(angPos.x, angPos.y, angPos.z, angPos.w),
            orientation_covariance = covariance,
            angular_velocity = new RosMessageTypes.Geometry.Vector3Msg(
                toNormalizedRad(angVel.eulerAngles.x),
                toNormalizedRad(angVel.eulerAngles.y),
                toNormalizedRad(angVel.eulerAngles.z)),
            angular_velocity_covariance = covariance,
            linear_acceleration = new RosMessageTypes.Geometry.Vector3Msg(acc.x, acc.y, acc.z),
            linear_acceleration_covariance = covariance
        };
        ros.Publish(topic, msg);

        // --- Publish heading (bearing from reference) ---
        try
        {
            // --- Heading calculation: IMUセンサー座標系Z+がワールドZ+（北）と一致している場合0度を出力 ---
            // ワールド座標系のZ+を北、X+を東とする
            // IMUのforward（センサー座標系Z+）をワールド座標系で取得
            Vector3 imuForwardWorld = transform.TransformDirection(Vector3.forward); // IMUのZ+がワールドでどこを向いているか
            // ワールド座標系の北（Z+）
            Vector3 worldNorth = Vector3.forward;
            // ワールド座標系の東（X+）
            Vector3 worldEast = Vector3.right;
            // 水平面に投影
            Vector3 imuForwardFlat = new Vector3(imuForwardWorld.x, 0, imuForwardWorld.z).normalized;
            Vector3 northFlat = new Vector3(worldNorth.x, 0, worldNorth.z).normalized;
            Vector3 eastFlat = new Vector3(worldEast.x, 0, worldEast.z).normalized;
            // 角度計算（北を0度、東を90度、時計回り）
            float heading = Vector3.SignedAngle(northFlat, imuForwardFlat, Vector3.up);
            // UnityのSignedAngleは左回りが正なので、時計回りに変換
            heading = (-heading + 360.0f) % 360.0f;
            // ノイズ付与
            float headingNoiseStdDev = 1.0f; // [deg]
            float noise = UnityEngine.Random.Range(-headingNoiseStdDev, headingNoiseStdDev);
            heading = (heading + noise + 360.0f) % 360.0f;
            ros.Publish(headingTopic, new Float64Msg(heading));
        }
        catch (Exception ex)
        {
            Debug.LogWarning($"Heading publish failed: {ex.Message}");
        }
    }

    private float toNormalizedRad(float angle)
    {
        // while (angle > 180.0f)
        // {
        //     angle -= 360.0f;
        // }
        // while (angle < -180.0f)
        // {
        //     angle += 360.0f;
        // }
        // return angle;
        float n = Mathf.Floor((angle + 180) / 360);
        angle -= n * 360;
        return angle * Mathf.Deg2Rad;
    }
}
