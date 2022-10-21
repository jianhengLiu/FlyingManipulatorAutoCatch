# sudo chmod 777 /dev/ttyACM0 & sleep 2;
roslaunch vins_estimator rs_camera.launch & sleep 10;
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600 & sleep 10;
roslaunch vins_estimator realsense_vio.launch
wait;
