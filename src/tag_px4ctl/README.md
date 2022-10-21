# TAG_PX4CTL

# auto_catch_ctl使用方法

1. NUC端启动硬件设备
   ```
   sh ./src/tag_px4ctl/scripts/rspx4_atlas.sh
   ```
2. Atlas端：启动状态反馈
   ```
    roslaunch vins_estimator realsense_vio_atlas.launch
   ```
3. NUC端：运行控制程序
   ```
    rosrun tag_px4ctl auto_catch_ctl
   ```
## 2. 遥控器

### 2.1. 通道说明：
- 5：控制控制模式
  - 低：手动
  - 中：位置模式
  - 高：伺服模式
- 6：控制飞控模式
  - 低：上锁
  - 中：
    - OFFBOARD模式下：降落
    - 其他模式：无操作
  - 高：解锁并切到OFFBOARD模式
- 7：急停开关
  - 高：急停

**进行一切操作时，务必一手握住通道7随时急停！！！**

### 2.2. 操作说明

1. 摇杆归零（通道567应当都为低档位），rpy归零
2. `rosrun tag_px4ctl auto_catch_ctl`
3. 5切中，提示油门增大到0.5
4. 油门增大到0.5，切到位置模式，此时应有`/mavros/setpoint_position/local`的消息发布，终端输出POSITION MODE
5. 6切高，无人机解锁并起飞
6. *推动rpt(hrottle)，无人机进行位置模式飞行
7. 5切高，setpoint设置为起飞位置往前1m处
8. POSITION MODE下若看到tag，则自动切换到tag伺服，此时无人机会自动飞到tag的零点上方
9. 5切中，setpoint设置为起飞位置处
10. 6切中，无人机降落
11. 6切低，无人机上锁

### 2.3. 应急操作

1. 先尝试6切中，令无人机自行降落，然后再6切低，令无人机上锁
2. 无人机若不降落，若有手飞经验，可5切低进行手动控制，注意当前油门为0.5，无人机可能会往上飞，适当控制油门自行降落，然后再6切低，令无人机上锁
3. 危急时刻，直接7切高，令无人机急停

# tag_servo_node使用方法
## 1. 终端
1. `roslaunch mavros px4.launch`
2. 启动摄像头

    `roslaunch vins_estimator rs_camera.launch`

    `roslaunch vins_estimator usb_cam-test.launch`
3. 启动状态估计
   
   `roslaunch tag_px4ctl mocap.launch`
   
   or 
   
   `roslaunch tag_px4ctl vins.launch`

4. 启动tag伺服（启动前请保证遥控器各个摇杆在初始位置）
   
   `rosrun tag_px4ctl tag_servo_node`

## 2. 遥控器

### 2.1. 通道说明：
- 5：控制控制模式
  - 低：手动
  - 中：位置模式
  - 高：伺服模式
- 6：控制飞控模式
  - 低：上锁
  - 中：
    - OFFBOARD模式下：降落
    - 其他模式：无操作
  - 高：解锁并切到OFFBOARD模式
- 7：急停开关
  - 高：急停

**进行一切操作时，务必一手握住通道7随时急停！！！**

### 2.2. 操作说明

1. 摇杆归零（通道567应当都为低档位），rpy归零
2. 启动tag伺服
   
   `rosrun tag_px4ctl tag_servo_node`
3. 5切中，提示油门增大到0.5
4. 油门增大到0.5，切到位置模式，此时应有`/mavros/setpoint_position/local`的消息发布
5. 6切高，无人机解锁并起飞
6. 推动rpt(hrottle)，无人机进行位置模式飞行
7. 若看到tag，5切高，则切换到tag伺服，此时无人机会自动飞到tag的零点上方；此时依旧可以通过油门控制高度
8. （5切中退出tag伺服）
9. 6切中，无人机降落
10. 6切低，无人机上锁

### 2.3. 应急操作

1. 先尝试6切中，令无人机自行降落，然后再6切低，令无人机上锁
2. 无人机若不降落，若有手飞经验，可5切低进行手动控制，注意当前油门为0.5，无人机可能会往上飞，适当控制油门自行降落，然后再6切低，令无人机上锁
3. 危急时刻，直接7切高，令无人机急停