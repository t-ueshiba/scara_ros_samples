# アカデミック スカラロボット ROS対応SDK
ヴイストン株式会社より発売されている水平多関節型ロボットアーム「アカデミック スカラロボット(VS-SR)」をROSで制御するためのSDKです。
別途Linux搭載のPC及びロボット実機が必要になります。

詳しい導入方法・取り扱いについては下記の資料をご参照ください。
https://www.vstone.co.jp/products/scara_robot/download/VSASR_ROS_Manual.pdf

### 準備
`.bashrc`に追加

```
###                                                                             
### USBXpressHostSDK settings                                                   
###                                                                             
if [ -d ${HOME}/USBXpressHostSDK ]; then                                        
  export USBXPRESSHOSTSDKHOME=${HOME}/USBXpressHostSDK/CP2110_4                 
  export PATH=${PATH}:${USBXPRESSHOSTSDKHOME}/bin                               
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${USBXPRESSHOSTSDKHOME}/lib/x86_64  
  export CPATH=${CPATH}:${USBXPRESSHOSTSDKHOME}/include                         
fi 

source ~/catkin_ws/devel/setup.bash
```
### テストプログラムの起動
`gazebo`によるシミュレーション
```
$ roslaunch scara_robot_samples gazebo_robot_controller.launch
```
実機による通信確認
```
$ roslaunch scara_robot_samples rviz_teaching.launch
```
GUIからの実機制御
```
$ roslaunch scara_robot_samples rviz_controller.launch
```
## 産総研オートメーション研究チームによる拡張
### テストプログラムの起動
`MoveIt`によるシミュレーション
```
$ roslaunch scara_robot_samples scara_robot_bringup.launch sim:=true
```
`MoveIt`による実機制御
```
$ roslaunch scara_robot_samples scara_robot_bringup.launch
```
### 対話型プログラムによる操作
最初に実機のドライバまたは`gazebo`（シミュレーション）を起動する。
```
$ roslaunch scara_robot_samples scara_robot_bringup.launch [sim:=true]
```
さらに、対話型操作プログラムを起動する。
```
$ roslaunch scara_robot_routines interactive.launch
```