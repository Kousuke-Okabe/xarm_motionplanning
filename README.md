# 環境構築
### 1. ワークスペース作成（任意）
> mkdir -p ~/xarm_ws/src    
> cd ~/xarm_ws  
> colcon build  
> source install/setup.bash

### 2. xarm_ros2のインストール
> cd src    
> git clone https://github.com/xArm-Developer/xarm_ros2.git

xarm_ros2(https://github.com/xArm-Developer/xarm_ros2)に従って環境インストール

### 3. リポジトリのクローン
> cd ~/xarm_ws/src  
> git clone https://github.com/Kousuke-Okabe/xarm_motionplanning.git　  
> cd ~/xarm_ws  
> colcon build  
> source install/setup.bash

# 実行方法
### Moveit 起動

シミュレータ
> ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py

実機
> ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.203

### 動作指令コマンド送信ノード
> ros2 launch xarm_motionplanning xarm_motionplanning.launch.py

# 参考
Move Group C++ Interfaceを用いて動作しているので，コード内容は下記を参照
https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html