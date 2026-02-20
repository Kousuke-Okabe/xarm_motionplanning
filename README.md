# 必要環境
* xarm_ros2
    > https://github.com/xArm-Developer/xarm_ros2.git

# 使い方
### Moveit 起動

シミュレータ
> ros2 launch xarm_moveit_config xarm6_moveit_fake.launch.py

実機
> ros2 launch xarm_moveit_config xarm6_moveit_realmove.launch.py robot_ip:=192.168.1.203

### 動作指令コマンド送信ノード
> ros2 launch xarm_motionplanning xarm_motionplanning.launch.py