# krb2026b_ros2
交流ロボコン2026BのROS 2

# 起動手順
opi ssh後 CANableを再接続

念の為マイコンリセットを行うと良い

```bash
ros2 launch krb2026b_launch real.launch.xml
```

で起動できる

手元の端末でrviz2で krb2026b.rviz を開き、自己位置などが問題ないかを確認する

RMCなどの基板が青く光らない場合はcanに問題がある

```bash
ip a
```

でCANやLiDARの確認を行える

```bash
candump can0
```
でCANを見れるがわかりずらいため以下を推奨する

```bash
candump can0 | python3 ~/robocon/src/natto_can_ros2/src/natto_can_bridge/include/natto_can_converter/natto_can_converter.py
```


# rosbag

## 保存

### 自己位置推定用データがほしいとき
```bash
ros2 bag record /sensing/lidar/laserscan/left_raw /sensing/lidar/laserscan/rear_raw /sensing/lidar/laserscan/right_raw /joint_states
```

センサーの生データと joint_states だけあればできる


## 解析

### 自己位置推定検証
```bash
ros2 launch krb2026b_launch rosbag_localization_test.launch.xml
```
を実行後
```bash
ros2 bag play rosbag名
```
で位置推定を動かすことができる

センシングも動くので、laserscanさえあればOK

## 編集

### rosbagの一部を切り出したいとき

```bash
python3 krb2026b_ros2/script/rosbag_cutter.py rosbag名
```
でカットできる

start offsetは開始時間

durationは開始時間から記録する時間

start offset + dutation = end time