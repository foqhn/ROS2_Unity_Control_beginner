using UnityEngine;
using UnityEngine.InputSystem; // 新しいInput Systemを使うためにこれを追加
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RosRoverController : MonoBehaviour
{
    public WheelCollider frontLeftWheelCollider;
    public WheelCollider frontRightWheelCollider;
    public WheelCollider rearLeftWheelCollider;
    public WheelCollider rearRightWheelCollider;

    public Transform frontLeftWheelTransform;
    public Transform frontRightWheelTransform;
    public Transform rearLeftWheelTransform;
    public Transform rearRightWheelTransform;

    public string topicName = "/rover/targets";
    public float wheelRadius = 0.085f; // 85mm = 0.085m
    public float maxMotorTorque = 3f;

    // PID parameters
    public float Kp = 10f;
    public float Ki = 0.5f;
    public float Kd = 0.1f;

    // Target wheel speeds [rad/s]
    private float[] targetSpeeds = new float[4];
    // PID state for each wheel
    private float[] prevErrors = new float[4];
    private float[] integrals = new float[4];

    // For actual wheel speed calculation
    private float[] currentSpeeds = new float[4];

    // For input handling
    private Vector2 moveInput;

    void Start()
    {
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<RosMessageTypes.Std.Float64MultiArrayMsg>(topicName, OnTargetsReceived);
        Debug.Log($"Subscribed to {topicName}");

        GetComponent<Rigidbody>().centerOfMass = new Vector3(0.0F, 0.01F, 0.0F);
    }

    void OnTargetsReceived(RosMessageTypes.Std.Float64MultiArrayMsg msg)
    {
        // 4要素ベクトル [FL, FR, RL, RR] (rad/s)
        for (int i = 0; i < 4; i++)
        {
            if (i < msg.data.Length)
                targetSpeeds[i] = (float)msg.data[i];
            else
                targetSpeeds[i] = 0f;
        }
    }

    void FixedUpdate()
    {
        // moveInput.y が前後入力 (W/Sキー)、moveInput.x が左右入力 (A/Dキー)
        float forward = moveInput.y; // 前後
        float turn = moveInput.x;    // 左右
        targetSpeeds[0] = forward * maxMotorTorque + turn * maxMotorTorque; // 前左
        targetSpeeds[1] = forward * maxMotorTorque - turn * maxMotorTorque; // 前右
        targetSpeeds[2] = forward * maxMotorTorque + turn * maxMotorTorque; // 後左
        targetSpeeds[3] = forward * maxMotorTorque - turn * maxMotorTorque; // 後右

        // 各WheelColliderの現在の回転速度(rad/s)を取得
        currentSpeeds[0] = frontLeftWheelCollider.rpm * 2f * Mathf.PI / 60f;
        currentSpeeds[1] = frontRightWheelCollider.rpm * 2f * Mathf.PI / 60f;
        currentSpeeds[2] = rearLeftWheelCollider.rpm * 2f * Mathf.PI / 60f;
        currentSpeeds[3] = rearRightWheelCollider.rpm * 2f * Mathf.PI / 60f;

        // PID制御でトルク計算
        float[] torques = new float[4];
        for (int i = 0; i < 4; i++)
        {
            float error = targetSpeeds[i] - currentSpeeds[i];
            integrals[i] += error * Time.fixedDeltaTime;
            float derivative = (error - prevErrors[i]) / Time.fixedDeltaTime;
            torques[i] = Kp * error + Ki * integrals[i] + Kd * derivative;
            torques[i] = Mathf.Clamp(torques[i], -maxMotorTorque, maxMotorTorque);
            prevErrors[i] = error;
        }

        frontLeftWheelCollider.motorTorque = torques[0];
        frontRightWheelCollider.motorTorque = torques[1];
        rearLeftWheelCollider.motorTorque = torques[2];
        rearRightWheelCollider.motorTorque = torques[3];

        frontLeftWheelCollider.steerAngle = 0f;
        frontRightWheelCollider.steerAngle = 0f;

        BrakeWheelIfStationary(frontLeftWheelCollider);
        BrakeWheelIfStationary(frontRightWheelCollider);
        BrakeWheelIfStationary(rearLeftWheelCollider);
        BrakeWheelIfStationary(rearRightWheelCollider);

        UpdateWheelVisuals(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelVisuals(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelVisuals(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelVisuals(rearRightWheelCollider, rearRightWheelTransform);
    }

    // 車輪間距離（トレッド幅）を取得（Transformから計算）
    float GetTreadWidth()
    {
        return Mathf.Abs(frontLeftWheelTransform.position.x - frontRightWheelTransform.position.x);
    }
    // PlayerInputが"Send Messages"モードの時、"Move"アクションが発生すると自動的にこのメソッドが呼ばれる
    // メソッド名は "On" + アクション名("Move") にする
    private void BrakeWheelIfStationary(WheelCollider wheel)
    {
        if (Mathf.Abs(wheel.motorTorque) < 0.05F)
        {
            wheel.brakeTorque = 1;
        }
        else
        {
            wheel.brakeTorque = 0;
        }
    }

    public void OnMove(InputValue value)
    {
        // 入力されたVector2の値を読み取って変数に保存する
        moveInput = value.Get<Vector2>();
    }
    //呼ばれない場合は、0に戻す

    void UpdateWheelVisuals(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 position;
        Quaternion rotation;
        wheelCollider.GetWorldPose(out position, out rotation);
        wheelTransform.position = position;
        wheelTransform.rotation = rotation;
    }
}
