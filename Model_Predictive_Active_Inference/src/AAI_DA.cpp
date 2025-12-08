#include "AAI_DA.h"
#include <chrono>

// 构造函数
AAI_DA::AAI_DA(ros::NodeHandle &nh)
    : px4_state(3),
      des_yaw(0.0),
      ground_truth_x(0.0), ground_truth_y(0.0), ground_truth_z(0.0), ground_truth_first_deri_x(0.0), ground_truth_first_deri_y(0.0), ground_truth_first_deri_z(0.0),
      ground_truth_x_car(0.0), ground_truth_y_car(0.0), ground_truth_z_car(0.0), ground_truth_first_deri_x_car(0.0), ground_truth_first_deri_y_car(0.0), ground_truth_first_deri_z_car(0.0),
      optimizer_xyz(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax)
{
  acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
  px4_state_sub = nh.subscribe("/px4_state_pub", 1, &AAI_DA::StateCallback,
                               this, ros::TransportHints().tcpNoDelay());
  relative_pos_sub = nh.subscribe("/point_with_fixed_delay", 1, &AAI_DA::relative_pos_Callback,
                                  this, ros::TransportHints().tcpNoDelay());
  ground_truth_sub = nh.subscribe("/mavros/local_position/velocity_local", 10, &AAI_DA::ground_truth_callback, this);
  ground_truth_pose_sub = nh.subscribe("/mavros/local_position/pose", 10, &AAI_DA::ground_truth_pose_callback, this);
  ground_truth_sub_car = nh.subscribe("/vrpn_client_node/Tracker0/0/twist", 10, &AAI_DA::ground_truth_callback_car, this);
  ground_truth_pose_sub_car = nh.subscribe("/vrpn_client_node/Tracker0/0/pose", 10, &AAI_DA::ground_truth_pose_callback_car, this);
  pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/aic2_xyz", 1);

  startControlLoops();
}

AAI_DA::~AAI_DA()
{
  // 确保控制线程安全退出
  if (xyz_thread.joinable())
    xyz_thread.join();
}

void AAI_DA::startControlLoops()
{
  xyz_thread = std::thread(&AAI_DA::xyzAxisControlLoop, this);
}

void AAI_DA::xyzAxisControlLoop()
{
  ros::Rate rate(50);
  std_msgs::Float64MultiArray hat_msg;
  hat_msg.data.assign(24, 0.0);
  const Eigen::Vector3d zero_acc = Eigen::Vector3d::Zero();

  acc_msg.position.x = acc_msg.position.y = acc_msg.position.z = 0;
  acc_msg.velocity.x = acc_msg.velocity.y = acc_msg.velocity.z = 0;
  acc_msg.jerk.x = acc_msg.jerk.y = acc_msg.jerk.z = 0;
  acc_msg.yaw_dot = 0;
  acc_msg.header.frame_id = "world";

  auto publishAcc = [&](const Eigen::Vector3d &acc)
  {
    acc_msg.acceleration.x = acc.x();
    acc_msg.acceleration.y = acc.y();
    acc_msg.acceleration.z = acc.z();
    acc_msg.yaw = des_yaw.load(std::memory_order_relaxed);
    acc_msg.header.stamp = ros::Time::now();
    acc_cmd_pub.publish(acc_msg);
  };

  auto publishHat = [&](const Eigen::Vector3d &acc)
  {
    hat_msg.data[0] = APOX.hat_x(0);
    hat_msg.data[1] = APOX.hat_x(1);
    hat_msg.data[2] = acc.x();
    hat_msg.data[3] = APOX.y_real;
    hat_msg.data[4] = ground_truth_x.load(std::memory_order_relaxed);
    hat_msg.data[5] = ground_truth_first_deri_x.load(std::memory_order_relaxed);
    hat_msg.data[6] = ground_truth_x_car.load(std::memory_order_relaxed);
    hat_msg.data[7] = ground_truth_first_deri_x_car.load(std::memory_order_relaxed);
    hat_msg.data[8] = APOY.hat_x(0);
    hat_msg.data[9] = APOY.hat_x(1);
    hat_msg.data[10] = acc.y();
    hat_msg.data[11] = APOY.y_real;
    hat_msg.data[12] = ground_truth_y.load(std::memory_order_relaxed);
    hat_msg.data[13] = ground_truth_first_deri_y.load(std::memory_order_relaxed);
    hat_msg.data[14] = ground_truth_y_car.load(std::memory_order_relaxed);
    hat_msg.data[15] = ground_truth_first_deri_y_car.load(std::memory_order_relaxed);
    hat_msg.data[16] = APOZ.hat_x(0);
    hat_msg.data[17] = APOZ.hat_x(1);
    hat_msg.data[18] = acc.z();
    hat_msg.data[19] = APOZ.y_real;
    hat_msg.data[20] = ground_truth_z.load(std::memory_order_relaxed);
    hat_msg.data[21] = ground_truth_first_deri_z.load(std::memory_order_relaxed);
    hat_msg.data[22] = ground_truth_z_car.load(std::memory_order_relaxed);
    hat_msg.data[23] = ground_truth_first_deri_z_car.load(std::memory_order_relaxed);
    pub_hat_x.publish(hat_msg);
  };

  while (ros::ok())
  {
    if (px4_state.load(std::memory_order_relaxed) == 3)
    {
      APOX.step();
      APOY.step();
      APOZ.step();
      auto start_time = std::chrono::high_resolution_clock::now();
      Vector3d xyz_real(APOX.hat_x(0), APOY.hat_x(0), APOZ.hat_x(0));
      Vector3d xyz_v_real(APOX.hat_x(1), APOY.hat_x(1), APOZ.hat_x(1));
      Vector3d optical_mu(optical_xyz[3], optical_xyz[4], optical_xyz[5]);
      Vector3d optical_mu_p(optical_xyz[6], optical_xyz[7], optical_xyz[8]);
      optical_xyz = optimizer_xyz.optimize(xyz_real, xyz_v_real, optical_mu, optical_mu_p);
      auto end_time = std::chrono::high_resolution_clock::now();
      double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
      ROS_INFO_STREAM_THROTTLE(1.0, "Execution time x: " << elapsed_time << " seconds");
      Eigen::Vector3d acc_cmd(optical_xyz[0], optical_xyz[1], optical_xyz[2]);
      publishAcc(acc_cmd);
      publishHat(acc_cmd);
    }
    else
    {
      publishAcc(zero_acc);
      publishHat(zero_acc);
    }
    rate.sleep();
  }
}

void AAI_DA::ground_truth_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  ground_truth_first_deri_x.store(msg->twist.linear.x, std::memory_order_relaxed);
  ground_truth_first_deri_y.store(msg->twist.linear.y, std::memory_order_relaxed);
  ground_truth_first_deri_z.store(msg->twist.linear.z, std::memory_order_relaxed);
}

void AAI_DA::ground_truth_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ground_truth_x.store(msg->pose.position.x, std::memory_order_relaxed);
  ground_truth_y.store(msg->pose.position.y, std::memory_order_relaxed);
  ground_truth_z.store(msg->pose.position.z, std::memory_order_relaxed);
}

void AAI_DA::ground_truth_callback_car(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  ground_truth_first_deri_x_car.store(msg->twist.linear.x, std::memory_order_relaxed);
  ground_truth_first_deri_y_car.store(msg->twist.linear.y, std::memory_order_relaxed);
  ground_truth_first_deri_z_car.store(msg->twist.linear.z, std::memory_order_relaxed);
}

void AAI_DA::ground_truth_pose_callback_car(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  ground_truth_x_car.store(msg->pose.position.x, std::memory_order_relaxed);
  ground_truth_y_car.store(msg->pose.position.y, std::memory_order_relaxed);
  ground_truth_z_car.store(msg->pose.position.z, std::memory_order_relaxed);
}

void AAI_DA::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  des_yaw.store(msg->data[5], std::memory_order_relaxed);
  APOX.pushMeasurement(msg->data[0], msg->data[4]);
  APOY.pushMeasurement(msg->data[1], msg->data[4]);
  APOZ.pushMeasurement(msg->data[2] + 0.8, msg->data[4]);
}

void AAI_DA::StateCallback(const std_msgs::Int32::ConstPtr &msg)
{
  px4_state.store(msg->data, std::memory_order_relaxed);
}
