# sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 & sleep 10;
roslaunch tag_px4ctl mocap.launch & sleep 2;
roslaunch tag_px4ctl usb_cam-test.launch
wait;
