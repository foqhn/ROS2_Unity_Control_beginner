using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using System;

public class RGB_Camera : MonoBehaviour
{
    public enum Resolution
    {
        R720p,
        R360p,
        R180p
    }

    public Resolution resolution = Resolution.R720p;
    public string rosTopicName = "/camera/image_raw";
    public Camera targetCamera;
    public int publishHz = 10;

    private ROSConnection ros;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private float timer = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(rosTopicName);

        Vector2Int res = GetResolution();
        renderTexture = new RenderTexture(res.x, res.y, 24);
        targetCamera.targetTexture = renderTexture;
        texture2D = new Texture2D(res.x, res.y, TextureFormat.RGB24, false);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer > 1f / publishHz)
        {
            timer = 0f;
            PublishImage();
        }
    }

    Vector2Int GetResolution()
    {
        switch (resolution)
        {
            case Resolution.R720p: return new Vector2Int(1280, 720);
            case Resolution.R360p: return new Vector2Int(640, 360);
            case Resolution.R180p: return new Vector2Int(320, 180);
            default: return new Vector2Int(1280, 720);
        }
    }

    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);

        texture2D.Apply();
        RenderTexture.active = null;

        // 画像を上下反転
        FlipTextureVertically(texture2D);

        byte[] imageData = texture2D.GetRawTextureData();
    // Texture2Dを上下反転
    void FlipTextureVertically(Texture2D tex)
    {
        int w = tex.width;
        int h = tex.height;
        Color[] pixels = tex.GetPixels();
        for (int y = 0; y < h / 2; y++)
        {
            for (int x = 0; x < w; x++)
            {
                int top = y * w + x;
                int bottom = (h - 1 - y) * w + x;
                Color temp = pixels[top];
                pixels[top] = pixels[bottom];
                pixels[bottom] = temp;
            }
        }
        tex.SetPixels(pixels);
        tex.Apply();
    }

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = GetCurrentRosTime(),
                frame_id = "camera"
            },
            height = (uint)texture2D.height,
            width = (uint)texture2D.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(texture2D.width * 3),
            data = imageData
        };
    // UnixエポックからのROS2タイムスタンプを生成
    RosMessageTypes.BuiltinInterfaces.TimeMsg GetCurrentRosTime()
    {
        var now = DateTime.UtcNow;
        var unixTime = (now - new DateTime(1970, 1, 1));
        uint sec = (uint)unixTime.TotalSeconds;
        uint nanosec = (uint)((unixTime.TotalSeconds - sec) * 1e9);
        return new RosMessageTypes.BuiltinInterfaces.TimeMsg((int)sec, nanosec);
    }

        ros.Publish(rosTopicName, msg);
    }
}