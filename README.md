# krb2026b_ros2
交流ロボコン2026BのROS 2

# 環境構築
OpenVINO

https://www.isus.jp/wp-content/uploads/openvino/2024/docs/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2024_3_0&OP_SYSTEM=LINUX&DISTRIBUTION=APT

humbleの場合は src/detection/krb2026b_duck_detection/include/krb2026b_duck_detection/duck_detection.hpp の

#include <cv_bridge/cv_bridge.hpp>

を

#include <cv_bridge/cv_bridge.h>

に

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

## mcapコマンドを使用したい場合

https://github.com/foxglove/mcap/releases?q=mcap-cli

ここからバイナリをダウンロードしたら使える

```bash
sudo chmod +x mcap-linux-amd64
sudo mv mcap-linux-amd64 /bin/mcap
```

## 保存

### 自己位置推定用データがほしいとき
```bash
ros2 bag record /sensing/lidar/laserscan/left_raw /sensing/lidar/laserscan/rear_raw /sensing/lidar/laserscan/right_raw /joint_states --compression-format zstd --compression-mode file
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