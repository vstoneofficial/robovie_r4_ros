## 研究開発用ロボットプラットフォーム Robovie-R4 ROS対応SDK
ヴイストン株式会社より発売されている開発研究用ロボットプラットフォーム「Robovie-R4」をROSで制御するためのSDKです。別途Linux搭載のPC及びロボット実機が必要になります。
Robovie-R4については、製品ページをご覧ください。

## 目次
- [1 準備](#1-準備)
  - [1.1 依存パッケージのインストール](#11-依存パッケージのインストール)
  - [1.2 メガローバーVer.3.0のROSパッケージをインストール](#12-メガローバーver30のrosパッケージをインストール)
- [2 robovie\_r4\_rosのセットアップ](#2-robovie_r4_rosのセットアップ)
  - [2.1 catkin ワークスペースの作成](#21-catkin-ワークスペースの作成)
  - [2.2 サンプルパッケージのクローン](#22-サンプルパッケージのクローン)
  - [2.3 依存パッケージのインストール](#23-依存パッケージのインストール)
  - [2.4 コードをビルド](#24-コードをビルド)
  - [2.5 オーバーレイ作業](#25-オーバーレイ作業)
- [3 各パッケージの説明と利用方法](#3-各パッケージの説明と利用方法)
    - [3.1 r4\_control](#31-r4_control)
    - [3.2 r4\_description](#32-r4_description)
    - [3.3 r4\_motion](#33-r4_motion)
    - [3.4 r4\_moveit\_config](#34-r4_moveit_config)
      - [3.4.1 MoveIt](#341-moveit)
      - [3.4.2 Gazeboシミュレータ](#342-gazeboシミュレータ)
- [4 Robovie-R4の制御方法](#4-robovie-r4の制御方法)
  - [4.1 ROS（ROS1）メッセージ通信](#41-rosros1メッセージ通信)
      - [Subscribe](#subscribe)
      - [Publish](#publish)
  - [4.2 MoveIt＋RVizで動かす](#42-moveitrvizで動かす)
- [5 変更履歴](#5-変更履歴)
    - [v1.0.0 (2023-03-15)](#v100-2023-03-15)

# 1 準備
ROSのインストール方法は[こちら](http://wiki.ros.org/noetic/Installation/Ubuntu)を確認してください。
## 1.1 依存パッケージのインストール
Robovie-R4 のパッケージ構築のための依存パッケージをインストールします。
```bash
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-moveit-visual-tools \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-rosparam-shortcuts \
  ros-noetic-rqt-joint-trajectory-controller \
  ros-noetic-rosserial \
  ros-noetic-rosserial-arduino \
  libgflags-dev
```

## 1.2 メガローバーVer.3.0のROSパッケージをインストール
Robovie-R4移動のための台車部分に研究開発用台車ロボットとして定評のある「[メガローバーVer.3.0](https://www.vstone.co.jp/products/wheelrobot/ver.3.0_normal.html)」を採用しています。

ROS上でRobovie-R4を動作したい場合、下記のパッケージが必要です。
```bash
git clone https://github.com/vstoneofficial/megarover3_ros.git  # メガローバーVer.3.0のパッケージ
git clone https://github.com/vstoneofficial/megarover_description.git  # メガローバーの3Dモデルのパッケージ
git clone -b noetic-devel https://github.com/PickNikRobotics/ros_control_boilerplate.git  # ROS_controlのハードウェアインターフェイスをセットアップするためのテンプレート
```

# 2 robovie_r4_rosのセットアップ
## 2.1 catkin ワークスペースの作成
[こちら](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)のチュートリアルを参照し、catkinワークスペースを作成します。下記の内容はワークスペースのディレクトリを`~/catkin_ws`
とします。
## 2.2 サンプルパッケージのクローン
```bash
cd ~/catkin_ws/src
git clone https://github.com/vstoneofficial/robovie_r4_ros.git
```

## 2.3 依存パッケージのインストール
Robovie-R4 のパッケージ構築のための依存パッケージをインストールします。
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 2.4 コードをビルド
```bash
cd ~/catkin_ws
catkin_make
```

## 2.5 オーバーレイ作業
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
既に`~/.bashrc`の中にある場合は、上記の操作をスキップして下記のコマンドを実行してください。
```bash
source ~/.bashrc
```

以上でrobovie_r4_rosパッケージのセットアップは完了です。\
次のコマンドを入力して、Robovie-R4をRViz上に表示します。
```bash
roslaunch r4_description display.launch
```

# 3 各パッケージの説明と利用方法
### 3.1 r4_control
Robovie-R4をROS Controlで制御するパッケージ。\
以下のコマンドでRobovie-R4をROSと接続。
```
roslaunch r4_control bringup.launch
```

以下のコマンドでRobovie-R4をros_controlで制御する
```
roslaunch r4_control r4_control_HW.launch
```

### 3.2 r4_description
Robovie-R4のモデルを表示するためのパッケージ。\
以下のコマンドで立ち上げます。
```
roslaunch r4_description display.launch
```
以下の画面が表示します。
![](images/r4-rviz.png)

### 3.3 r4_motion
Robovie-R4のサンプルモーション（MoveItへ指令を出すノード）のパッケージ。\
各ノードの詳細は`src`フォルダーの各ファイルに記載されています。

### 3.4 r4_moveit_config
Robovie-R4　MoveItの設定パッケージ
#### 3.4.1 MoveIt
以下のコマンドでRViz上でMoveItを操作するデモを立ち上げてください。
```
roslaunch r4_moveit_config demo.launch
```
![](images/r4-moveit.png)

#### 3.4.2 Gazeboシミュレータ
以下のコマンドでGazeboでRobovie-R4シミュレーションを起動します。
```
roslaunch r4_moveit_config　gazebo.launch
```
MoveItで操作するGazeboのシミュレーションは次のコマンドで立ち上げてください。
```
roslaunch r4_moveit_config　demo_gazebo.launch
```
> **Note**
> パラメータチューニングが必要な場合はあります。

# 4 Robovie-R4の制御方法
## 4.1 ROS（ROS1）メッセージ通信
Robovie-R4 の subscribe（購読）するメッセージ、publish（配信）するメッセージの仕様について解説します。
#### Subscribe
- /r4/r4Cmd
  - 内容
      ```
      int16[11] angle # 各軸の目標角度の指令値 [0.1 deg]
      uint16[3] led # 目のLED（R, G, B）
      int16 time # 目標角度までの遷移時間 [16.667 msec]
      ```
      
#### Publish
- /r4/r4Sensor
  - 内容
      ```
      int16[11] angle # 各軸の現在角度 [0.1 deg]
      ```
      

## 4.2 MoveIt＋RVizで動かす
上記の [3.1 r4_control](#31-r4_control) のコマンドを実行してから、GUI上の操作が可能です。
MoveItの使い方の詳細は[公式のドキュメント](https://ros-planning.github.io/moveit_tutorials/index.html)を参照してください。


# 5 変更履歴
### v1.0.0 (2023-03-15)
- 初版