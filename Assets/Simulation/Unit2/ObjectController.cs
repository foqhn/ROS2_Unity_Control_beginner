using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // TwistMsgはこの中に含まれます

public class ObjectController : MonoBehaviour
{
    // ROSのトピック名（Twistでは/cmd_velが慣例的）
    public string topicName = "/cmd_vel";

    // 受信した速度情報を保持する変数
    private Vector3 linearVelocity = Vector3.zero;
    private Vector3 angularVelocity = Vector3.zero;

    void Start()
    {
        // ROSコネクションのインスタンスを取得
        ROSConnection ros = ROSConnection.GetOrCreateInstance();

        // 指定したトピック名の購読を開始
        // メッセージの型を <TwistMsg> に変更
        ros.Subscribe<TwistMsg>(topicName, UpdateVelocity);
        Debug.Log($"Subscribing to {topicName}");
    }

    // トピックを受信したときに呼び出されるコールバック関数
    void UpdateVelocity(TwistMsg message)
    {
        // 受信した線形速度を格納
        linearVelocity = new Vector3((float)message.linear.x, (float)message.linear.y, (float)message.linear.z);

        // 受信した角速度を格納
        // ROS(右手系)のヨー軸(Z軸周り)はUnity(左手系)のY軸周りの回転に相当し、向きが逆になるため-1を掛ける
        angularVelocity = new Vector3((float)message.angular.x, -(float)message.angular.y, (float)message.angular.z);
    }

    // 毎フレーム呼び出されるUpdate関数
    // ここでオブジェクトの実際の移動・回転処理を行う
    void Update()
    {
        // Time.deltaTimeを掛けることで、フレームレートに依存しない動きにする
        
        // 線形速度に基づいて、オブジェクト自身の座標系で移動させる
        transform.Translate(linearVelocity * Time.deltaTime, Space.Self);

        // 角速度に基づいて、オブジェクトを回転させる
        transform.Rotate(angularVelocity * Time.deltaTime, Space.Self);
    }
}