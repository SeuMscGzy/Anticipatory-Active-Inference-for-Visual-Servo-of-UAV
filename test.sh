sudo chmod 777 /dev/ttyACM0 & sleep 1;
#source ~/test_ws/devel/setup.bash 
roscore & sleep 1;

#动捕节点
#roslaunch vrpn_client_ros sample.launch server:=192.168.0.200 & sleep 2;

#前部相机节点（与上述节点选择一个启动）
#rosrun img_detect img_detect & sleep 1;

#订阅动捕数据得到里程计
#rosrun motion_cap_to_odom motion_cap_to_odom & sleep 2;

#mavrso节点 并提升imu数据频率
#roslaunch mavros px4.launch & sleep 2;
#roslaunch mavros px4.launch gcs_url:=udp://@192.168.0.101 & sleep 3;
#rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
#rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
#rosrun mavros mavcmd long 511 32 10000 0 0 0 0 0 & sleep 1;

#将视觉定位发送给px4飞控
#rosrun fake_pose_publisher fake_pose_pub & sleep 1;
#rosrun topic_tools relay /vrpn_client_node/MCServer/0/pose /mavros/vision_pose/pose & sleep 2;


#切换无人机跟踪偏置的节点
#rosrun Keyboard_change_bias Keyboard_change_bias & sleep 1;
rosrun RSM2 RSM2 & sleep 1;
#rosrun Relative_Pos_Cal Relative_Pos_Cal & sleep 1;
rosrun Second_order_system Second_order_system & sleep 1;
#数据记录节点
rosrun record_curves record_curves & sleep 1;

rosrun Model_Predictive_Active_Inference testnode_mpai & sleep 1;
#rosrun video_creater video_creater & sleep 1;

#无人机offboard状态机节点
#roslaunch px4ctrl run_ctrl.launch & sleep 1
wait;