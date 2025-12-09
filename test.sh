#source ~/test_ws/devel/setup.bash 

#动捕节点
roslaunch vrpn_client_ros sample.launch server:=192.168.10.4 & sleep 2;

#mavrso节点 并提升imu数据频率
#roslaunch mavros px4.launch & sleep 2;
#roslaunch mavros px4.launch gcs_url:=udp://@192.168.66.206 & sleep 3;
roslaunch mavros px4.launch gcs_url:=udp://@192.168.10.3 & sleep 3;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 32 5000 0 0 0 0 0 & sleep 1;

#rosrun Relative_Pos_Cal set_d405_preset & sleep 5;

#将视觉定位发送给px4飞控
rosrun fake_pose_publisher fake_pose_pub & sleep 1;
rosrun topic_tools relay /vrpn_client_node/Tracker1/1/pose /mavros/vision_pose/pose & sleep 2;


#rosrun Relative_Pos_Cal Relative_Pos_Cal & sleep 2;

#rosrun Model_Predictive_Active_Inference testnode_mpai & sleep 1;

#无人机offboard状态机节点
roslaunch px4ctrl run_ctrl.launch & sleep 1;

#rosrun RAID_AgiVS_for_car_tracking RAIDAgiVS & sleep 1;

#python3 mpc_acados/scripts/main.py & sleep 1;

#数据记录节点
#rosrun record_curves record_curves & sleep 1;

#rosrun video_creater video_creater & sleep 1;
wait;