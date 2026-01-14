#include "AAI_DA.h"
#include <chrono>

// 构造函数
AAI_DA::AAI_DA(ros::NodeHandle &nh)
    : des_yaw(0.0), motor1(0), motor2(0), motor3(0), motor4(0), q_x(0.0), q_y(0.0), q_z(1.0), q_w(0.0), T_total(0.0),
      ground_truth_x(0.0), ground_truth_y(0.0), ground_truth_z(0.0), ground_truth_first_deri_x(0.0), ground_truth_first_deri_y(0.0), ground_truth_first_deri_z(0.0),
      ground_truth_x_car(0.0), ground_truth_y_car(0.0), ground_truth_z_car(0.0), ground_truth_first_deri_x_car(0.0), ground_truth_first_deri_y_car(0.0), ground_truth_first_deri_z_car(0.0),
      optimizer_xyz(dt, Np, precice_z1, precice_z2, precice_w1, precice_w2, precice_z_u, e1, umin, umax)
{
  acc_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
  relative_pos_sub = nh.subscribe("/point_with_fixed_delay", 1, &AAI_DA::relative_pos_Callback,
                                  this, ros::TransportHints().tcpNoDelay());
  ground_truth_sub = nh.subscribe("/vrpn_client_node/GZY0/0/velocity", 10, &AAI_DA::ground_truth_callback, this);
  ground_truth_pose_sub = nh.subscribe("/vrpn_client_node/GZY0/0/pose", 10, &AAI_DA::ground_truth_pose_callback, this);
  ground_truth_sub_car = nh.subscribe("/vrpn_client_node/GZY1/0/velocity", 10, &AAI_DA::ground_truth_callback_car, this);
  ground_truth_pose_sub_car = nh.subscribe("/vrpn_client_node/GZY1/0/pose", 10, &AAI_DA::ground_truth_pose_callback_car, this);
  motor_speed_sub = nh.subscribe("/mavros/esc_status", 10, &AAI_DA::escStatusCb, this);
  odom_sub_ = nh.subscribe<sensor_msgs::Imu>(
      "/mavros/imu/data", 200, // 队列建议大一点，方便插值
      &AAI_DA::odomCallback, this,
      ros::TransportHints().tcpNoDelay());
  pub_hat_x = nh.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 1);
  excel_update_timer = nh.createTimer(ros::Duration(0.01), &AAI_DA::controlUpdate, this);

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
  ros::Rate rate(40);
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

  while (ros::ok())
  {
    APOX.step();
    APOY.step();
    APOZ.step();
    // auto start_time = std::chrono::high_resolution_clock::now();
    Vector3d xyz_real(APOX.hat_x(0), APOY.hat_x(0), APOZ.hat_x(0));
    Vector3d xyz_v_real(APOX.hat_x(1), APOY.hat_x(1), APOZ.hat_x(1));
    Vector3d optical_mu(optical_xyz[3], optical_xyz[4], optical_xyz[5]);
    Vector3d optical_mu_p(optical_xyz[6], optical_xyz[7], optical_xyz[8]);
    optical_xyz = optimizer_xyz.optimize(xyz_real, xyz_v_real, optical_mu, optical_mu_p);
    // auto end_time = std::chrono::high_resolution_clock::now();
    // double elapsed_time = std::chrono::duration<double>(end_time - start_time).count();
    // ROS_INFO_STREAM_THROTTLE(1.0, "Execution time x: " << elapsed_time << " seconds");
    Eigen::Vector3d acc_cmd(optical_xyz[0], optical_xyz[1], optical_xyz[2]);
    publishAcc(acc_cmd);
    // publishHat(acc_cmd);
    rate.sleep();
  }
}

void AAI_DA::odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  q_w = msg->orientation.w;
  q_x = msg->orientation.x;
  q_y = msg->orientation.y;
  q_z = msg->orientation.z;
  // convert quaternion (q_w, q_x, q_y, q_z) to rotation matrix
  Eigen::Quaterniond quat(q_w, q_x, q_y, q_z);
  quat.normalize();
  R_world_from_body = quat.toRotationMatrix();
  // R_world_from_body can now be used to transform vectors from body to world frame
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

void AAI_DA::escStatusCb(const mavros_msgs::ESCStatus::ConstPtr &msg)
{
  motor1 = msg->esc_status[0].rpm; // 电机 i 的转速 (RPM)
  motor2 = msg->esc_status[1].rpm;
  motor3 = msg->esc_status[2].rpm;
  motor4 = msg->esc_status[3].rpm;
  constexpr double k1 = 5.7725e-05;
  constexpr double k2 = -0.0228;
  constexpr double k3 = -11.098;

  // rpm -> rad/s
  double omega1 = motor1 * 2.0 * M_PI / 60.0;
  double omega2 = motor2 * 2.0 * M_PI / 60.0;
  double omega3 = motor3 * 2.0 * M_PI / 60.0;
  double omega4 = motor4 * 2.0 * M_PI / 60.0;

  // thrust_i = k1*omega_i^2 + k2*omega_i + k3
  double thrust1 = k1 * omega1 * omega1 + k2 * omega1 + k3;
  double thrust2 = k1 * omega2 * omega2 + k2 * omega2 + k3;
  double thrust3 = k1 * omega3 * omega3 + k2 * omega3 + k3;
  double thrust4 = k1 * omega4 * omega4 + k2 * omega4 + k3;
  T_total = thrust1 + thrust2 + thrust3 + thrust4;
  Eigen::Vector3d thrust_body(0.0, 0.0, T_total);
  Eigen::Vector3d thrust_world = R_world_from_body * thrust_body - Eigen::Vector3d(0.0, 0.0, 9.81 * 0.54); // 减去重力
  APOX.u_thr = thrust_world(0);
  APOY.u_thr = thrust_world(1);
  APOZ.u_thr = thrust_world(2);
}

void AAI_DA::relative_pos_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  des_yaw.store(msg->data[5], std::memory_order_relaxed);
  img_x = msg->data[0];
  img_y = msg->data[1];
  img_z = msg->data[2];
  lost = msg->data[4];
  /*APOX.pushMeasurement(ground_truth_x_car.load(std::memory_order_relaxed) - ground_truth_x.load(std::memory_order_relaxed), msg->data[4]);
  APOY.pushMeasurement(ground_truth_y_car.load(std::memory_order_relaxed) - ground_truth_y.load(std::memory_order_relaxed), msg->data[4]);
  APOZ.pushMeasurement(ground_truth_z_car.load(std::memory_order_relaxed) - ground_truth_z.load(std::memory_order_relaxed) + 0.8, msg->data[4]);*/
  APOX.pushMeasurement(img_x, msg->data[4]);
  APOY.pushMeasurement(img_y, msg->data[4]);
  APOZ.pushMeasurement(img_z + 1.2, msg->data[4]);
}

void AAI_DA::controlUpdate(const ros::TimerEvent &)
{
  std_msgs::Float64MultiArray hat_msg;
  hat_msg.data.assign(33, 0.0);
  auto publishHat = [&]()
  {
    hat_msg.data[0] = APOX.hat_x(0);
    hat_msg.data[1] = APOX.hat_x(1);
    hat_msg.data[2] = optical_xyz[0];
    hat_msg.data[3] = img_x;
    hat_msg.data[4] = ground_truth_x.load(std::memory_order_relaxed);
    hat_msg.data[5] = ground_truth_first_deri_x.load(std::memory_order_relaxed);
    hat_msg.data[6] = ground_truth_x_car.load(std::memory_order_relaxed);
    hat_msg.data[7] = ground_truth_first_deri_x_car.load(std::memory_order_relaxed);
    hat_msg.data[8] = APOY.hat_x(0);
    hat_msg.data[9] = APOY.hat_x(1);
    hat_msg.data[10] = optical_xyz[1];
    hat_msg.data[11] = img_y;
    hat_msg.data[12] = ground_truth_y.load(std::memory_order_relaxed);
    hat_msg.data[13] = ground_truth_first_deri_y.load(std::memory_order_relaxed);
    hat_msg.data[14] = ground_truth_y_car.load(std::memory_order_relaxed);
    hat_msg.data[15] = ground_truth_first_deri_y_car.load(std::memory_order_relaxed);
    hat_msg.data[16] = APOZ.hat_x(0);
    hat_msg.data[17] = APOZ.hat_x(1);
    hat_msg.data[18] = optical_xyz[2];
    hat_msg.data[19] = img_z;
    hat_msg.data[20] = ground_truth_z.load(std::memory_order_relaxed);
    hat_msg.data[21] = ground_truth_first_deri_z.load(std::memory_order_relaxed);
    hat_msg.data[22] = ground_truth_z_car.load(std::memory_order_relaxed);
    hat_msg.data[23] = ground_truth_first_deri_z_car.load(std::memory_order_relaxed);
    hat_msg.data[24] = double(lost);
    hat_msg.data[25] = double(motor1);
    hat_msg.data[26] = double(motor2);
    hat_msg.data[27] = double(motor3);
    hat_msg.data[28] = double(motor4);
    hat_msg.data[29] = q_w;
    hat_msg.data[30] = q_x;
    hat_msg.data[31] = q_y;
    hat_msg.data[32] = q_z;
    pub_hat_x.publish(hat_msg);
  };
  publishHat();
}
