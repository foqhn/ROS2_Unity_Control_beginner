# ROS2 Unity Control Simulator

UnityとROS2を組み合わせたシミュレータプロジェクト

## 概要
このプロジェクトは、Unityの物理演算とレンダリング機能を活用し、ROS2と通信してロボットの制御やセンサデータの取得を行うシミュレーション環境を提供します。

## 前提条件
*   Unity (バージョン: 6000.1.12f1)
*   ROS2 (Humble)
*   ROS-TCP-Endpoint (ROS2側で必要)

## セットアップ
1.  このリポジトリをクローンします。
    ```bash
    git clone <repository-url>
    ```
2.  Unity Hubからプロジェクトを開きます。
3.  必要なUnityパッケージが自動的に解決されるのを待ちます。
