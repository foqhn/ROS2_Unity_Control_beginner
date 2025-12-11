using UnityEngine;
using UnityEngine.InputSystem; // 新しいInput Systemを使うためにこれを追加

public class CarController : MonoBehaviour
{
    // --- WheelColliderとTransformの変数はそのまま ---
    public WheelCollider frontLeftWheelCollider;
    public WheelCollider frontRightWheelCollider;
    public WheelCollider rearLeftWheelCollider;
    public WheelCollider rearRightWheelCollider;

    public Transform frontLeftWheelTransform;
    public Transform frontRightWheelTransform;
    public Transform rearLeftWheelTransform;
    public Transform rearRightWheelTransform;
    
    public float motorTorque = 100f;
    public float maxSteerAngle = 30f;

    // --- 入力値を保存するための変数を追加 ---
    private Vector2 moveInput;

    // FixedUpdateは物理演算を行う場所なのでそのまま
    void FixedUpdate()
    {
        // 古いInput.GetAxisの代わりに、保存した入力値を使う
        // moveInput.y が前後入力 (W/Sキー)、moveInput.x が左右入力 (A/Dキー)
        float forward = moveInput.y; // 前後
        float turn = moveInput.x;    // 左右

        // スキッドステア（差動）制御
        // 左右の車輪に異なるトルクを与えることで旋回
        float leftPower = forward * motorTorque + turn * motorTorque;
        float rightPower = forward * motorTorque - turn * motorTorque;

        // 前輪・後輪とも同じトルクを与える
        frontLeftWheelCollider.motorTorque = leftPower;
        rearLeftWheelCollider.motorTorque = leftPower;
        frontRightWheelCollider.motorTorque = rightPower;
        rearRightWheelCollider.motorTorque = rightPower;

        // スキッドステアではステアリング角は使わない
        frontLeftWheelCollider.steerAngle = 0f;
        frontRightWheelCollider.steerAngle = 0f;

        UpdateWheelVisuals(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelVisuals(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelVisuals(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelVisuals(rearRightWheelCollider, rearRightWheelTransform);
    }

    // PlayerInputが"Send Messages"モードの時、"Move"アクションが発生すると自動的にこのメソッドが呼ばれる
    // メソッド名は "On" + アクション名("Move") にする
    public void OnMove(InputValue value)
    {
        // 入力されたVector2の値を読み取って変数に保存する
        moveInput = value.Get<Vector2>();
    }

    void UpdateWheelVisuals(WheelCollider wheelCollider, Transform wheelTransform)
    {
        Vector3 position;
        Quaternion rotation;
        wheelCollider.GetWorldPose(out position, out rotation);

        wheelTransform.position = position;
        wheelTransform.rotation = rotation;
    }
}