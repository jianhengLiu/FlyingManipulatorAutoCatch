# sudo chmod 777 /dev/ttyACM0 & sleep 2;
export ROS_HOSTNAME=192.168.1.223   #主机IP 
export ROS_MASTER_URI=http://192.168.1.223:11311
roslaunch vins_estimator rs_camera.launch & sleep 5;
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 & sleep 5;
roslaunch tag_px4ctl usb_cam-test.launch
wait;
