#include <yaml-cpp/yaml.h>
#include "lidarodom.h"

namespace zjloc
{
#define USE_ANALYTICAL_DERIVATE 1 //    是否使用解析求导

     lidarodom::lidarodom(/* args */)
     {
          laser_point_cov = 0.001;
          current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

          CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          CT_ICP::CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          index_frame = 1;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());

          R_align = computeGravityAlignment(g_odom, g_imu);
     }

     lidarodom::~lidarodom()
     {
     }

     void lidarodom::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);
          options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
          options_.log_print = yaml["odometry"]["log_print"].as<bool>();
          options_.max_num_iteration = yaml["odometry"]["max_num_iteration"].as<int>();

          options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
          options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
          options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
          options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
          options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
          options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
          options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
          options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
          options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
          options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
          options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
          options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
          options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
          options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
          options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

          options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
          options_.ratio_of_nonground = yaml["odometry"]["ratio_of_nonground"].as<double>();
          options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
          std::string str_motion_compensation = yaml["odometry"]["motion_compensation"].as<std::string>();
          if (str_motion_compensation == "NONE")
               options_.motion_compensation = MotionCompensation::NONE;
          else if (str_motion_compensation == "CONSTANT_VELOCITY")
               options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
          else if (str_motion_compensation == "ITERATIVE")
               options_.motion_compensation = MotionCompensation::ITERATIVE;
          else if (str_motion_compensation == "CONTINUOUS")
               options_.motion_compensation = MotionCompensation::CONTINUOUS;
          else
               std::cout << "The `motion_compensation` " << str_motion_compensation << " is not supported." << std::endl;

          std::string str_icpmodel = yaml["odometry"]["icpmodel"].as<std::string>();
          if (str_icpmodel == "POINT_TO_PLANE")
               options_.icpmodel = POINT_TO_PLANE;
          else if (str_icpmodel == "CT_POINT_TO_PLANE")
               options_.icpmodel = CT_POINT_TO_PLANE;
          else
               std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;

          options_.beta_location_consistency = yaml["odometry"]["beta_location_consistency"].as<double>();
          options_.beta_orientation_consistency = yaml["odometry"]["beta_orientation_consistency"].as<double>();
          options_.beta_constant_velocity = yaml["odometry"]["beta_constant_velocity"].as<double>();
          options_.beta_small_velocity = yaml["odometry"]["beta_small_velocity"].as<double>();
          options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
          options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();
     }

     bool lidarodom::init(const std::string &config_yaml)
     {
          config_yaml_ = config_yaml;
          StaticIMUInit::Options imu_init_options;
          imu_init_options.use_speed_for_static_checking_ = false; // 本节数据不需要轮速计
          imu_init_ = StaticIMUInit(imu_init_options);

          text = "No Degeneracy";

          auto yaml = YAML::LoadFile(config_yaml_);
          delay_time_ = yaml["delay_time"].as<double>();
          // lidar和IMU外参
          std::vector<double> ext_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["extrinsic_R"] << std::endl;
          // std::cout << "lidar_R_wrt_IMU1:\n"
          //           << lidar_R_wrt_IMU << std::endl;

          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          // std::cout << "q_IL : [w, x, y, z] = [" 
          // << q_IL.w() << ", " 
          // << q_IL.x() << ", " 
          // << q_IL.y() << ", " 
          // << q_IL.z() << "]" << std::endl;

          q_IL.normalized();
          // std::cout << "q_IL (normalized): [w, x, y, z] = [" 
          // << q_IL.w() << ", " 
          // << q_IL.x() << ", " 
          // << q_IL.y() << ", " 
          // << q_IL.z() << "]" << std::endl;

          lidar_R_wrt_IMU = q_IL;
          // lidar_R_wrt_IMU = q_IL.toRotationMatrix();
          // lidar_R_wrt_IMU.transposeInPlace();
          // lidar_T_wrt_IMU = -lidar_R_wrt_IMU * lidar_T_wrt_IMU;

          // Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          // q_IL.normalized();

          // std::cout << "lidar_R_wrt_IMU2:\n"
          //           << lidar_R_wrt_IMU << std::endl;

          // init TIL
          TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
          // TIL_ = SE3(lidar_R_wrt_IMU, lidar_T_wrt_IMU);
          
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;

          CT_ICP::LidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::LidarPlaneNormFactor::q_il = TIL_.rotationMatrix();
          CT_ICP::CTLidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::CTLidarPlaneNormFactor::q_il = TIL_.rotationMatrix();

          loadOptions();
          switch (options_.motion_compensation)
          {
          case NONE:
          case CONSTANT_VELOCITY:
               options_.point_to_plane_with_distortion = false;
               options_.icpmodel = POINT_TO_PLANE;
               break;
          case ITERATIVE:
               options_.point_to_plane_with_distortion = true;
               options_.icpmodel = POINT_TO_PLANE;
               break;
          case CONTINUOUS:
               options_.point_to_plane_with_distortion = true;
               options_.icpmodel = CT_POINT_TO_PLANE;
               break;
          }
          LOG(WARNING) << "motion_compensation:" << options_.motion_compensation << ", model: " << options_.icpmodel;

          return true;
     }

     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data)
     {
          if (data.first < last_timestamp_lidar_)
          {
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }
     void lidarodom::pushData(IMUPtr imu)
     {
          double timestamp = imu->timestamp_;
          if (timestamp < last_timestamp_imu_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               imu_buffer_.clear();
          }

          last_timestamp_imu_ = timestamp;

          mtx_buf.lock();
          imu_buffer_.emplace_back(imu);
          mtx_buf.unlock();
          cond.notify_one();
     }
     void lidarodom::pushData(const nav_msgs::Odometry::ConstPtr& odometryMsg)
     {
       std::lock_guard<std::mutex> lock2(odoLock);
       odomQueue.push_back(*odometryMsg);

     //  std::cout << "\033[2K\rCurrent odomQueue size: " << odomQueue.size() << std::flush;

     //   const size_t maxQueueSize = 10000; // 设置最大队列大小
     //   if (odomQueue.size() > maxQueueSize) {
     //       odomQueue.pop_front(); // 移除最早的数据
     //      }
     }

     void lidarodom::pushData(cv::Mat img, double timestamp)
     {
          if(timestamp < last_timestamp_img_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               img_buffer_.clear();
          }
          last_timestamp_img_ = timestamp;

          mtx_buf.lock();
          img_buffer_.emplace_back(std::move(img));
          img_time_buffer_.emplace_back(timestamp);
          mtx_buf.unlock();
          cond.notify_one();
     }

     /**
      * [功能描述]：激光雷达里程计主运行函数，负责循环处理传感器测量数据
      * 该函数是整个系统的核心运行循环，会持续等待并处理来自激光雷达、IMU等传感器的数据
      * 通过多线程同步机制确保数据处理的时序性和完整性
      */
     void lidarodom::run()
     {
          // 主处理循环 - 系统会一直运行直到程序结束
          while (true)
          {
               // 存储待处理的测量数据组，每个组包含时间同步的多传感器数据
               std::vector<MeasureGroup> measurements;
               
               // 获取缓冲区互斥锁，确保线程安全访问共享数据
               std::unique_lock<std::mutex> lk(mtx_buf);
               
               // 调试输出：等待条件变量前的状态
               // std::cout << "Before waiting for condition..." << std::endl;
               
               // 条件变量等待：阻塞当前线程直到有可处理的测量数据
               // 当getMeasureMents()返回的数据大小不为0时，条件满足，线程被唤醒
               cond.wait(lk, [&]
                         { return (measurements = getMeasureMents()).size() != 0; });
               
               // 调试输出：显示获取到的测量数据数量
               // std::cout << "Condition met, measurements size: " << measurements.size() << std::endl;
               
               // 释放互斥锁，允许其他线程访问缓冲区
               lk.unlock();

               // 逐个处理获取到的测量数据组
               for (auto &m : measurements)
               {
                    // 调试输出：开始处理当前帧
                    // std::cout << "Processing frame: " << std::endl;
                    
                    // 使用计时器评估测量数据处理的性能
                    // ProcessMeasurements函数执行传感器数据融合和位姿估计
                    zjloc::common::Timer::Evaluate([&]()
                                                  { ProcessMeasurements(m); },
                                                  "processMeasurement");

                    // 性能监控代码块：计算数据处理速度
                    {
                         // 获取当前实际时间（高精度时钟）
                         auto real_time = std::chrono::high_resolution_clock::now();
                         
                         // 静态变量：保存上一次记录的实际时间，用于计算时间差
                         static std::chrono::system_clock::time_point prev_real_time = real_time;

                         // 每5秒统计一次处理速度
                         if (real_time - prev_real_time > std::chrono::seconds(5))
                         {
                              // 获取当前处理的数据时间戳（激光雷达帧结束时间）
                              auto data_time = m.lidar_end_time_;
                              
                              // 静态变量：保存上一次记录的数据时间戳
                              static double prev_data_time = data_time;
                              
                              // 计算实际经过的时间（秒）
                              auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                              
                              // 计算仿真/数据时间的变化量（秒）
                              auto delta_sim = data_time - prev_data_time;
                              
                              // 计算并输出处理速度倍率（注释掉的代码）
                              // 该倍率表示处理rosbag的速度相对于实时的倍数
                              // printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                              // 更新记录的时间戳，为下次计算做准备
                              prev_data_time = data_time;
                              prev_real_time = real_time;
                         }
                    }
               }
          }
     }

     /**
      * [功能描述]：处理传感器测量数据的核心函数，执行SLAM的主要算法流程
      * 该函数负责融合激光雷达、IMU和相机数据，进行状态预测、位姿估计和观测更新
      * 同时处理退化场景下的传感器切换逻辑，确保系统的鲁棒性
      * @param meas：包含时间同步的多传感器测量数据组，包括激光雷达点云、IMU数据和图像
      */
     void lidarodom::ProcessMeasurements(MeasureGroup &meas)
     {
          // 调试输出：开始处理测量数据
          // std::cout << "开始测试"<< std::endl;
          
          // 将当前测量数据组保存到成员变量中，供其他函数使用
          measures_ = meas;

          // 检查IMU是否需要初始化
          if (imu_need_init_)
          {
               // 如果IMU未初始化，则尝试初始化IMU并直接返回
               TryInitIMU();
               return;
          }

          // 调试输出相关代码（已注释）
          // std::cout << ANSI_DELETE_LAST_LINE;
          // std::cout << ANSI_COLOR_GREEN << "============== process frame: "
          //           << index_frame << ANSI_COLOR_RESET << std::endl;
          
          // 清空IMU状态缓存，为当前帧的处理做准备
          imu_states_.clear(); // 每次处理新帧时都需要清空

          // ==================== 状态预测阶段 ====================
          // 使用IMU数据进行状态预测，基于运动模型推算当前时刻的状态
          zjloc::common::Timer::Evaluate([&]()
                                             { Predict(); },
                                             "predict");
          
          // ==================== 状态初始化阶段 ====================
          // 初始化当前帧的状态（位置、姿态等），为优化提供初值
          zjloc::common::Timer::Evaluate([&]()
                                             { stateInitialization(); },
                                             "state init");

          // ==================== 点云数据准备 ====================
          // 复制激光雷达点云数据，避免修改原始数据
          std::vector<point3D> const_surf;
          const_surf.insert(const_surf.end(), meas.lidar_.begin(), meas.lidar_.end());
          // 备选方案：const_surf.assign(meas.lidar_.begin(), meas.lidar_.end());

          // ==================== 点云帧构建 ====================
          cloudFrame *p_frame;
          
          // 旧版本：仅使用激光雷达数据构建帧（已注释）
          // zjloc::common::Timer::Evaluate([&]()
          //                                { p_frame = buildFrame(const_surf, current_state,
          //                                                       meas.lidar_begin_time_,
          //                                                       meas.lidar_end_time_); },
          //                                "build frame");
          
          // 新版本：同时使用激光雷达和相机数据构建帧
          zjloc::common::Timer::Evaluate([&]()
                                             { p_frame = buildFrame(const_surf, meas.img_, current_state,
                                                                 meas.lidar_begin_time_,
                                                                 meas.lidar_end_time_); },
                                             "build frame");

          // ==================== 位姿估计阶段 ====================
          // 执行激光雷达里程计（LIO）优化，估计精确的位姿
          zjloc::common::Timer::Evaluate([&]()
                                             { poseEstimation(p_frame); },
                                             "poseEstimate");

          // ==================== 观测更新阶段 ====================
          // 将LIO的结果作为观测值，更新ESKF滤波器
          SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);
          
          // 调试输出：观测值和预测值对比（已注释）
          // std::cout << "obs: " << current_state->translation.transpose() << ", " << current_state->rotation.transpose() << std::endl;
          // SE3 pred_pose = eskf_.GetNominalSE3();
          // std::cout << "pred: " << pred_pose.translation().transpose() << ", " << pred_pose.so3().log().transpose() << std::endl;
          
          // 使用LIO位姿观测更新ESKF，协方差分别为1e-2（位置和姿态）
          zjloc::common::Timer::Evaluate([&]()
                                             { eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2); },
                                             "eskf_obs");

          // ==================== 数据发布和融合逻辑 ====================
          zjloc::common::Timer::Evaluate([&]()
                                             {
               // 定义ROS话题名称
               std::string laser_topic = "laser";      // 主要位姿话题
               std::string laser_topic2 = "orin_laser"; // LIO位姿话题
               std::string laser_topic3 = "orin_vins";  // VIO位姿话题
               
               // 静态变量：用于记录Z轴起始位置（当前未使用）
               static double z_axis_start = 0.0; 
               static bool z_axis_recorded = false; 

               // ==================== 外部里程计数据处理 ====================
               // 如果外部里程计队列不为空，获取最接近当前时间的里程计数据
               if (!odomQueue.empty()){
                    nav_msgs::Odometry externalOdom = getClosestOdom(meas.lidar_end_time_);

                    // 转换ROS四元数到tf四元数
                    tf::Quaternion orientation;
                    tf::quaternionMsgToTF(externalOdom.pose.pose.orientation, orientation);
                    
                    // 提取外部里程计的姿态和位置
                    Eigen::Quaterniond odom_quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
                    Eigen::Vector3d odom_trans(
                              externalOdom.pose.pose.position.x,
                              externalOdom.pose.pose.position.y,
                              externalOdom.pose.pose.position.z
                         );

                    // 应用重力对齐变换，将外部里程计数据转换到当前坐标系
                    external_pose = SE3(Eigen::Quaterniond(R_align * odom_quat.toRotationMatrix()), R_align * odom_trans);
               }

               // ==================== 退化场景处理逻辑 ====================
               
               // 情况1：首次检测到退化且当前仍在退化状态
               if (first_is_degenerate && is_degenerate) {
                    // 首次进入退化时，计算VIO到LIO的变换关系
                    if (first_degenerate) {   
                         text = "Switch to VIO";  // 状态文本：切换到VIO
                         VIO_to_LIO = external_pose.inverse() * pose_of_lo_;  // 计算变换矩阵
                         first_degenerate = false;  // 标记已处理首次退化
                    }
                    // 使用VIO数据和变换关系计算融合位姿
                    fused_pose = external_pose * VIO_to_LIO;
                    pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               } 
               // 情况2：首次检测到退化但当前已退出退化状态
               else if (first_is_degenerate && !is_degenerate) {
                    // 首次退出退化时，计算LIO到融合位姿的变换关系
                    if (first_exit_degenerate) {
                         text = "Switch to LIO";  // 状态文本：切换到LIO
                         LIO_to_Fused = Last_pose_of_lo_.inverse() * fused_pose;  // 计算变换矩阵
                         first_exit_degenerate = false;  // 标记已处理首次退出退化
                    }
                    // 使用LIO数据和变换关系计算融合位姿
                    fused_pose = SE3(LIO_to_Fused.rotationMatrix() * pose_of_lo_.rotationMatrix(),
                                        pose_of_lo_.translation() + LIO_to_Fused.translation());
                    pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               }

               // 情况3：非首次检测但当前处于退化状态
               if (!first_is_degenerate && is_degenerate) {
                    // 首次进入退化时，计算变换偏移
                    if (first_degenerate) {
                         text = "Switch to VIO";  // 状态文本：切换到VIO
                         
                         // 分别计算位置和姿态偏移
                         Eigen::Vector3d last_translation = Last_external_pose.translation();
                         Eigen::Matrix3d last_rotation = Last_external_pose.rotationMatrix();
                         Eigen::Vector3d fused_translation = fused_pose.translation();
                         Eigen::Matrix3d fused_rotation = fused_pose.rotationMatrix();
                         
                         // 计算变换偏移量
                         Eigen::Vector3d translation_offset = fused_translation - last_translation;
                         Eigen::Matrix3d rotation_offset = last_rotation.inverse() * fused_rotation;
                         
                         VIO_to_Fused = SE3(rotation_offset, translation_offset);
                         first_degenerate = false;  // 标记已处理首次退化
                    }
                    // 使用VIO数据和偏移计算融合位姿
                    fused_pose = SE3(external_pose.rotationMatrix() * VIO_to_Fused.rotationMatrix(),
                                        external_pose.translation() + VIO_to_Fused.translation());
                    pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               } 
               // 情况4：非首次检测且当前未退化但曾经进入过退化状态
               else if (!first_is_degenerate && !is_degenerate && has_entered_degenerate) {
                    // 首次退出退化时，计算LIO到融合位姿的变换偏移
                    if (first_exit_degenerate) {
                         text = "Switch to LIO";  // 状态文本：切换到LIO
                         
                         // 分别计算位置和姿态偏移
                         Eigen::Vector3d last_translation = Last_pose_of_lo_.translation();
                         Eigen::Matrix3d last_rotation = Last_pose_of_lo_.rotationMatrix();
                         Eigen::Vector3d fused_translation = fused_pose.translation();
                         Eigen::Matrix3d fused_rotation = fused_pose.rotationMatrix();
                         
                         // 计算变换偏移量
                         Eigen::Vector3d translation_offset = fused_translation - last_translation;
                         Eigen::Matrix3d rotation_offset = last_rotation.inverse() * fused_rotation;
                         
                         LIO_to_Fused = SE3(rotation_offset, translation_offset);
                         first_exit_degenerate = false;  // 标记已处理首次退出退化
                    }
                    // 使用LIO数据和偏移计算融合位姿
                    fused_pose = SE3(pose_of_lo_.rotationMatrix() * LIO_to_Fused.rotationMatrix(),
                                        pose_of_lo_.translation() + LIO_to_Fused.translation());
                    pub_pose_to_ros(laser_topic, fused_pose, meas.lidar_end_time_);
               }

               // ==================== 状态记录 ====================
               // 保存当前位姿，用于下一帧的变换计算
               Last_pose_of_lo_ = pose_of_lo_;
               Last_external_pose = external_pose;

               // ==================== 数据发布 ====================
               // 如果从未进入退化状态，直接发布LIO位姿
               if (!entered_degenerate) 
                    pub_pose_to_ros(laser_topic, pose_of_lo_, meas.lidar_end_time_);
               
               // 发布原始LIO位姿到专用话题
               pub_pose_to_ros(laser_topic2, pose_of_lo_, meas.lidar_end_time_);
               
               // 发布外部VIO位姿到专用话题
               pub_pose_to_ros(laser_topic3, external_pose, meas.lidar_end_time_);
               
               // 发布速度信息
               laser_topic = "velocity";
               SE3 pred_pose = eskf_.GetNominalSE3();  // 获取预测位姿
               Eigen::Vector3d vel_world = eskf_.GetNominalVel();  // 获取世界坐标系速度
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse() * vel_world;  // 转换到机体坐标系
               pub_data_to_ros(laser_topic, vel_base.x(), text);  // 发布X方向速度
               
               // 发布状态文本信息
               laser_topic = "text";
               pub_data_to_ros(laser_topic, 0, text);
               
               // 每8帧发布一次累计距离信息
               if(index_frame % 8 == 0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();  // 上次位置记录
                    Eigen::Vector3d t = pred_pose.translation();  // 当前位置
                    static double dist = 0;  // 累计距离
                    dist += (t - last_t).norm();  // 累加距离增量
                    last_t = t;  // 更新位置记录
                    pub_data_to_ros(laser_topic, dist, text);  // 发布累计距离
               } 
          }, "pub cloud");  // 计时器名称

          // ==================== 状态保存和内存管理 ====================
          // 为当前帧保存状态副本
          p_frame->p_state = new state(current_state, true);
          
          // 保存状态到历史记录（注释的版本会消耗大量内存）
          // all_cloud_frame.push_back(p_frame); // TODO: 保存这个，特别费内存
          
          // 创建状态副本并保存到历史记录
          state *tmp_state = new state(current_state, true);
          all_state_frame.push_back(tmp_state);
          
          // 为下一帧创建新的当前状态
          current_state = new state(current_state, false);

          // ==================== 清理工作 ====================
          index_frame++;  // 帧计数器递增
          p_frame->release();  // 释放帧资源
          
          // 清空临时容器，释放内存
          std::vector<point3D>().swap(meas.lidar_);
          std::vector<point3D>().swap(const_surf);
     }

     /**
      * [功能描述]：激光雷达里程计位姿估计的主函数
      * 该函数是位姿估计流程的核心入口，负责协调优化、地图更新和视场管理三个关键步骤
      * 通过非线性优化算法精确估计当前帧的位姿，并维护局部地图的一致性和有效性
      * @param p_frame：当前点云帧指针，包含待处理的激光雷达数据和状态信息
      */
     void lidarodom::poseEstimation(cloudFrame *p_frame)
     {
          // TODO: 检查current_state数据的有效性（待实现的功能）
          // 未来可以在此处添加状态数据的完整性和合理性检查
          
          // ==================== 位姿优化阶段 ====================
          // 跳过第一帧的优化处理，因为第一帧没有足够的历史信息进行配准
          // 从第二帧开始执行基于点云配准的位姿优化算法
          if (index_frame > 1)
          {
               // 使用计时器评估优化过程的性能开销
               // optimize函数执行点到平面的ICP配准优化，精确估计位姿参数
               zjloc::common::Timer::Evaluate([&]()
                                             { optimize(p_frame); },
                                             "optimize");
          }

          // ==================== 地图更新控制 ====================
          // 控制变量：决定是否将当前帧的点云数据添加到局部地图中
          // 设置为true表示启用增量式地图构建
          bool add_points = true;
          
          // ==================== 增量式地图更新 ====================
          // 如果启用地图更新，则将优化后的点云数据集成到局部地图中
          if (add_points)
          { 
               // 地图更新注释：在此处更新局部地图
               // 使用计时器评估地图更新过程的性能开销
               // map_incremental函数负责将当前帧的点云增量添加到体素地图中
               // 采用体素化方法管理地图数据，确保内存效率和查询速度
               zjloc::common::Timer::Evaluate([&]()
                                             { map_incremental(p_frame); },
                                             "map update");
          }

          // ==================== 视场范围管理 ====================
          // 使用计时器评估视场分割过程的性能开销
          // lasermap_fov_segment函数负责维护局部地图的有效范围
          // 移除超出传感器有效距离的远距离点云，控制地图大小和计算复杂度
          // 这是一种滑动窗口式的地图管理策略，确保系统的实时性能
          zjloc::common::Timer::Evaluate([&]()
                                             { lasermap_fov_segment(); },
                                             "fov segment");
     }

     /**
      * [功能描述]：激光雷达里程计的核心优化函数
      * 该函数使用基于Ceres的非线性优化算法，通过点到平面的ICP配准精确估计位姿
      * 支持连续时间ICP（CT-ICP）和传统ICP两种模式，并包含多种正则化约束确保优化稳定性
      * 同时检测和处理环境退化情况，提高系统在挑战性场景下的鲁棒性
      * @param p_frame：当前点云帧指针，包含待优化的激光雷达数据和初始状态估计
      */
     void lidarodom::optimize(cloudFrame *p_frame)
     {
          // ==================== 历史状态信息获取 ====================
          // 初始化前一帧状态相关变量，用于添加时间一致性约束
          state *previous_state = nullptr;
          Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();  // 前一帧位置
          Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();     // 前一帧速度
          Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();  // 前一帧姿态

          // ==================== 当前帧状态初始化 ====================
          // 获取当前帧的状态指针和初始位姿估计
          state *curr_state = p_frame->p_state;
          
          // 提取帧开始和结束时刻的姿态四元数（用于CT-ICP）
          Eigen::Quaterniond begin_quat = Eigen::Quaterniond(curr_state->rotation_begin);
          Eigen::Quaterniond end_quat = Eigen::Quaterniond(curr_state->rotation);
          
          // 提取帧开始和结束时刻的位置向量（用于CT-ICP）
          Eigen::Vector3d begin_t = curr_state->translation_begin;
          Eigen::Vector3d end_t = curr_state->translation;

          // ==================== 前一帧状态信息提取 ====================
          // 如果不是第一帧，则获取前一帧的状态信息用于时间一致性约束
          if (p_frame->frame_id > 1)
          {
               // 调试输出：显示状态帧数量和当前帧ID
               if (options_.log_print)
                    std::cout << "all_cloud_frame.size():" << all_state_frame.size() << ", " << p_frame->frame_id << std::endl;
               
               // 获取前一帧的状态（索引为当前帧ID-2，因为索引从0开始）
               // previous_state = all_cloud_frame[p_frame->frame_id - 2]->p_state;  // 旧版本使用点云帧历史
               previous_state = all_state_frame[p_frame->frame_id - 2];  // 新版本使用状态帧历史
               
               // 提取前一帧的关键信息
               previous_translation = previous_state->translation;  // 前一帧结束位置
               previous_velocity = previous_state->translation - previous_state->translation_begin;  // 前一帧运动速度
               previous_orientation = previous_state->rotation;  // 前一帧结束姿态
          }
          
          // 调试输出：显示前一帧和当前帧的位置信息
          if (options_.log_print)
          {
               std::cout << "prev end: " << previous_translation.transpose() << std::endl;
               std::cout << "curr begin: " << p_frame->p_state->translation_begin.transpose()
                         << "\ncurr end: " << p_frame->p_state->translation.transpose() << std::endl;
          }

          // ==================== 关键点采样 ====================
          // 对表面点进行网格采样，减少计算量同时保持特征完整性
          std::vector<point3D> surf_keypoints;
          gridSampling(p_frame->point_surf, surf_keypoints,
                         options_.sampling_rate * options_.surf_res);

          // 记录原始点云大小（用于调试）
          size_t num_size = p_frame->point_surf.size();

          // ==================== 点云变换函数定义 ====================
          // Lambda函数：根据优化后的位姿参数变换关键点到世界坐标系
          auto transformKeypoints = [&](std::vector<point3D> &point_frame)
          {
               Eigen::Matrix3d R;  // 旋转矩阵
               Eigen::Vector3d t;  // 平移向量
               
               // 遍历所有关键点进行坐标变换
               for (auto &keypoint : point_frame)
               {
                    // ==================== CT-ICP模式：考虑运动畸变 ====================
                    // 如果启用运动补偿或使用连续时间ICP模型
                    if (options_.point_to_plane_with_distortion ||
                         options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
                    {
                         // 获取该点的时间插值系数（0表示帧开始，1表示帧结束）
                         double alpha_time = keypoint.alpha_time;

                         // 使用球面线性插值（SLERP）计算该时刻的姿态
                         Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
                         q.normalize();  // 归一化四元数
                         R = q.toRotationMatrix();  // 转换为旋转矩阵
                         
                         // 使用线性插值计算该时刻的位置
                         t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
                    }
                    // ==================== 传统ICP模式：不考虑运动畸变 ====================
                    else
                    {
                         // 直接使用帧结束时刻的位姿
                         R = end_quat.normalized().toRotationMatrix();
                         t = end_t;
                    }
                    
                    // 应用坐标变换：激光雷达坐标系 -> IMU坐标系 -> 世界坐标系
                    keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
               }
          };

          // ==================== 主优化循环 ====================
          // 迭代优化，逐步精确位姿估计
          for (int iter(0); iter < options_.max_num_iteration; iter++)
          {
               // ==================== 点云变换 ====================
               // 使用当前位姿估计变换关键点
               transformKeypoints(surf_keypoints);

               // ==================== Ceres优化问题设置 ====================
               // 设置Huber损失函数，对异常值具有鲁棒性
               // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));  // 备选参数
               ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
               
               // 创建Ceres优化问题
               ceres::Problem::Options problem_options;
               ceres::Problem problem(problem_options);
               
               // 选择四元数参数化方式
          #ifdef USE_ANALYTICAL_DERIVATE
               // 使用自定义的旋转参数化（解析导数）
               ceres::LocalParameterization *parameterization = new RotationParameterization();
          #else
               // 使用Eigen四元数参数化（自动微分）
               auto *parameterization = new ceres::EigenQuaternionParameterization();
          #endif

               // ==================== 参数块添加 ====================
               // 根据ICP模型类型添加不同的优化参数
               switch (options_.icpmodel)
               {
               case IcpModel::CT_POINT_TO_PLANE:
                    // CT-ICP模式：优化帧开始和结束的位姿（6个参数块）
                    problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);  // 开始姿态
                    problem.AddParameterBlock(&end_quat.x(), 4, parameterization);    // 结束姿态
                    problem.AddParameterBlock(&begin_t.x(), 3);                       // 开始位置
                    problem.AddParameterBlock(&end_t.x(), 3);                         // 结束位置
                    break;
               case IcpModel::POINT_TO_PLANE:
                    // 传统ICP模式：仅优化帧结束的位姿（2个参数块）
                    problem.AddParameterBlock(&end_quat.x(), 4, parameterization);    // 结束姿态
                    problem.AddParameterBlock(&end_t.x(), 3);                         // 结束位置
                    break;
               }

               // ==================== 点到平面距离残差项 ====================
               // 构建表面点的点到平面距离约束
               std::vector<ceres::CostFunction *> surfFactor;  // 存储代价函数
               std::vector<Eigen::Vector3d> normalVec;         // 存储平面法向量
               addSurfCostFactor(surfFactor, normalVec, surf_keypoints, p_frame);

               // 检查是否为最后一次迭代和提前退出条件
               bool is_last_iteration = (iter == options_.max_num_iteration - 1);
               bool is_exit_condition_met = false;

               // ==================== 添加残差块到优化问题 ====================
               int surf_num = 0;  // 残差计数器
               if (options_.log_print)
                    std::cout << "get factor: " << surfFactor.size() << std::endl;
                    
               // 逐个添加表面点残差
               for (auto &e : surfFactor)
               {
                    surf_num++;
                    switch (options_.icpmodel)
                    {
                    case IcpModel::CT_POINT_TO_PLANE:
                         // CT-ICP：残差依赖于开始和结束位姿
                         problem.AddResidualBlock(e, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
                         break;
                    case IcpModel::POINT_TO_PLANE:
                         // 传统ICP：残差仅依赖于结束位姿
                         problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
                         break;
                    }
                    // 可选：限制最大残差数量（当前已注释）
                    // if (surf_num > options_.max_num_residuals)
                    //      break;
               }
                         
               // 释放临时容器内存
               // std::vector<Eigen::Vector3d>().swap(normalVec);  // 暂时不释放，后续需要用于退化检测
               std::vector<ceres::CostFunction *>().swap(surfFactor);

               // ==================== 正则化约束项（仅CT-ICP模式）====================
               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
               {
                    // ==================== 位置一致性约束 ====================
                    // 约束当前帧开始位置与前一帧结束位置的差异
                    if (options_.beta_location_consistency > 0.)
                    {
          #ifdef USE_ANALYTICAL_DERIVATE
                         // 使用解析导数的位置一致性因子
                         CT_ICP::LocationConsistencyFactor *cost_location_consistency =
                              new CT_ICP::LocationConsistencyFactor(previous_translation, sqrt(surf_num * options_.beta_location_consistency * laser_point_cov));
          #else
                         // 使用自动微分的位置一致性因子
                         auto *cost_location_consistency =
                              CT_ICP::LocationConsistencyFunctor::Create(previous_translation, sqrt(surf_num * options_.beta_location_consistency));
          #endif
                         problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());
                    }

                    // ==================== 姿态一致性约束 ====================
                    // 约束当前帧开始姿态与前一帧结束姿态的差异
                    if (options_.beta_orientation_consistency > 0.)
                    {
          #ifdef USE_ANALYTICAL_DERIVATE
                         // 使用解析导数的姿态一致性因子
                         CT_ICP::RotationConsistencyFactor *cost_rotation_consistency =
                              new CT_ICP::RotationConsistencyFactor(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency * laser_point_cov));
          #else
                         // 使用自动微分的姿态一致性因子
                         auto *cost_rotation_consistency =
                              CT_ICP::OrientationConsistencyFunctor::Create(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency));
          #endif
                         problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());
                    }

                    // ==================== 小速度约束 ====================
                    // 约束帧内运动速度，防止过度运动
                    if (options_.beta_small_velocity > 0.)
                    {
          #ifdef USE_ANALYTICAL_DERIVATE
                         // 使用解析导数的小速度因子
                         CT_ICP::SmallVelocityFactor *cost_small_velocity =
                              new CT_ICP::SmallVelocityFactor(sqrt(surf_num * options_.beta_small_velocity * laser_point_cov));
          #else
                         // 使用自动微分的小速度因子
                         auto *cost_small_velocity =
                              CT_ICP::SmallVelocityFunctor::Create(sqrt(surf_num * options_.beta_small_velocity));
          #endif
                         problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());
                    }

                    // ==================== 常速度约束（已注释）====================
                    // 可选：约束当前帧速度与前一帧速度的一致性
                    // if (options_.beta_constant_velocity > 0.)
                    // {
                    //      CT_ICP::VelocityConsistencyFactor2 *cost_velocity_consistency =
                    //          new CT_ICP::VelocityConsistencyFactor2(previous_velocity, sqrt(surf_num * options_.beta_constant_velocity * laser_point_cov));
                    //      problem.AddResidualBlock(cost_velocity_consistency, nullptr, PR_begin, PR_end);
                    // }
               }

               // ==================== 残差数量检查 ====================
               // 确保有足够的约束进行优化
               if (surf_num < options_.min_num_residuals)
               {
                    std::stringstream ss_out;
                    ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
                    ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
                    std::cout << "ERROR: " << ss_out.str();
               }

               // ==================== Ceres求解器配置 ====================
               ceres::Solver::Options options;
               options.max_num_iterations = 5;  // 最大内部迭代次数
               options.num_threads = 3;         // 并行线程数
               options.minimizer_progress_to_stdout = false;  // 不打印进度
               options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;  // LM算法

               // 备选求解器配置（已注释）
               // ceres::Solver::Options options;
               // options.linear_solver_type = ceres::DENSE_SCHUR;
               // options.trust_region_strategy_type = ceres::DOGLEG;
               // options.max_num_iterations = 10;
               // options.minimizer_progress_to_stdout = false;
               // options.num_threads = 6;

               // ==================== 执行优化求解 ====================
               ceres::Solver::Summary summary;
               ceres::Solve(options, &problem, &summary);

               // 检查求解是否成功
               if (!summary.IsSolutionUsable())
               {
                    std::cout << summary.FullReport() << std::endl;
                    throw std::runtime_error("Error During Optimization");
               }

               // ==================== 四元数归一化 ====================
               // 确保四元数的单位长度约束
               begin_quat.normalize();
               end_quat.normalize();

               // ==================== 收敛性检查 ====================
               // 计算位姿变化量，判断是否收敛
               double diff_trans = 0, diff_rot = 0;
               
               // 计算位置变化量
               diff_trans += (current_state->translation_begin - begin_t).norm();
               diff_trans += (current_state->translation - end_t).norm();
               
               // 计算姿态变化量（角距离）
               diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);
               diff_rot += AngularDistance(current_state->rotation, end_quat);

               // 检查是否满足收敛条件
               if (diff_rot < options_.thres_orientation_norm &&
                    diff_trans < options_.thres_translation_norm)
               {
                    // 满足收敛条件，标记可以提前退出
                    // if (options_.log_print)
                    //      std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                    // break;
                    is_exit_condition_met = true;
               }

               // ==================== 退化检测（在最后一次迭代或收敛时）====================
               if (is_last_iteration || is_exit_condition_met) {
                    // 使用平面法向量检测环境的可定位性
                    // if (checkLocalizability(surf_keypoints, normalVec) == 1)  // 旧版本同时检查关键点
                    if (checkLocalizability(normalVec) == 1) 
                    {
                         // 检测到退化情况
                         is_degenerate = true;          // 当前状态为退化
                         entered_degenerate = true;     // 标记已进入退化状态
                         
                         // 处理首次退化的标志设置
                         if (!prev_is_degenerate) {
                              if (!has_entered_degenerate) {
                              first_is_degenerate = true;      // 系统首次检测到退化
                              has_entered_degenerate = true;   // 系统已经历退化
                              } else {
                              first_is_degenerate = false;     // 非首次进入退化状态
                              }
                              first_degenerate = true;  // 首次从正常状态进入退化状态
                              
                              // 调试输出（已注释）
                              // std::cout << "Setting first_degenerate to true: "
                              //           << "prev_is_degenerate: " << prev_is_degenerate
                              //           << ", has_entered_degenerate: " << has_entered_degenerate
                              //           << std::endl;
                         }
                    }
                    else {
                         // 未检测到退化
                         is_degenerate = false;
                         if (prev_is_degenerate) {
                              first_exit_degenerate = true;  // 首次从退化状态退出
                         }
                    }
                    prev_is_degenerate = is_degenerate;  // 更新前一状态记录
               }
               
               // 释放法向量内存
               std::vector<Eigen::Vector3d>().swap(normalVec);

               // ==================== 状态更新 ====================
               // 将优化后的参数更新到状态变量中
               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
               {
                    // CT-ICP模式：更新帧开始和结束的完整状态
                    p_frame->p_state->translation_begin = begin_t;
                    p_frame->p_state->rotation_begin = begin_quat;
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation_begin = begin_t;
                    current_state->translation = end_t;
                    current_state->rotation_begin = begin_quat;
                    current_state->rotation = end_quat;
               }
               if (options_.icpmodel == IcpModel::POINT_TO_PLANE)
               {
                    // 传统ICP模式：仅更新帧结束状态
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation = end_t;
                    current_state->rotation = end_quat;
               }

               // ==================== 收敛退出检查 ====================
               // 如果满足收敛条件，则提前退出优化循环
               if (is_exit_condition_met) {
                    if (options_.log_print)
                         std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                    break;
               }
          }     

          // ==================== 清理和调试输出 ====================
          // 释放关键点内存
          std::vector<point3D>().swap(surf_keypoints);
          
          // 调试输出：显示优化后的位姿
          if (options_.log_print)
          {
               std::cout << "opt: " << p_frame->p_state->translation_begin.transpose()
                         << ",end: " << p_frame->p_state->translation.transpose() << std::endl;
          }

          // ==================== 最终点云变换 ====================
          // 使用优化后的位姿变换所有表面点，为地图更新做准备
          transformKeypoints(p_frame->point_surf);
     }

     /**
      * [功能描述]：根据给定的时间戳，从里程计数据队列中找到时间最接近的里程计消息
      * @param timestamp：目标时间戳，用于查找最接近时刻的里程计数据
      * @return 返回nav_msgs::Odometry类型的里程计消息，包含位置、姿态和速度信息
      */
     nav_msgs::Odometry lidarodom::getClosestOdom(double timestamp) {
          // std::lock_guard<std::mutex> lock(odoLock);  // 被注释的互斥锁，用于多线程安全
     
          // 初始化最接近的里程计消息变量
          nav_msgs::Odometry closestOdom;
          // 初始化最小时间差为最大可能值
          double minTimeDiff = std::numeric_limits<double>::max();
     
          // 被注释掉的线性搜索实现（时间复杂度O(n)）
          // for (const auto& odom : odomQueue) {
          //     double timeDiff = std::abs(odom.header.stamp.toSec() - timestamp);
          //     if (timeDiff < minTimeDiff) {
          //         minTimeDiff = timeDiff;
          //         closestOdom = odom;
          //     }
          // }

          // 检查里程计队列是否为空，如果为空则抛出异常
          if (odomQueue.empty()) {
               throw std::runtime_error("odomQueue is empty");
          }
     
          // 使用二分查找算法在有序队列中查找目标时间戳位置（时间复杂度O(log n)）
          // lower_bound返回第一个不小于目标时间戳的元素位置
          auto it = std::lower_bound(odomQueue.begin(), odomQueue.end(), timestamp,
                                   [](const nav_msgs::Odometry& odom, double time) {
                                        return odom.header.stamp.toSec() < time;  // 比较函数：里程计时间戳小于目标时间
                                   });
     
          // 处理边界情况：如果迭代器指向队列末尾，说明目标时间戳大于所有里程计时间
          if (it == odomQueue.end()) {
               closestOdom = odomQueue.back();  // 返回队列中最后一个（时间最新的）里程计数据
          }
          // 处理边界情况：如果迭代器指向队列开头，说明目标时间戳小于所有里程计时间
          if (it == odomQueue.begin()) {
               closestOdom = odomQueue.front();  // 返回队列中第一个（时间最早的）里程计数据
          } else {
               // 一般情况：目标时间戳在队列时间范围内，需要比较前后两个里程计数据的时间差
               auto prev_it = std::prev(it);  // 获取前一个里程计数据的迭代器
               
               // 比较前一个数据和当前数据与目标时间戳的绝对时间差
               if (std::abs(prev_it->header.stamp.toSec() - timestamp) <
               std::abs(it->header.stamp.toSec() - timestamp)) {
               closestOdom = *prev_it;  // 前一个数据更接近目标时间
               } else {
               closestOdom = *it;       // 当前数据更接近目标时间
               }
          }

          return closestOdom;  // 返回找到的最接近时间戳的里程计数据
     }

     Eigen::Matrix3d lidarodom::computeGravityAlignment(const Eigen::Vector3d& g_odom, const Eigen::Vector3d& g_imu) const {

          Eigen::Vector3d axis = g_imu.cross(g_odom).normalized();
          double angle = acos(g_imu.dot(g_odom) / (g_imu.norm() * g_odom.norm()));
      
          Eigen::AngleAxisd rotation_vector(angle, axis);
          return rotation_vector.toRotationMatrix();
     }

     /**
      * [功能描述]：检查系统的可定位性，通过分析平面法向量的分布来判断是否处于退化场景
      * @param planeNormals：平面法向量集合，包含从点云中提取的平面特征的法向量
      * @return 返回值为double类型：1表示检测到退化场景，需要使用外部里程计；0表示场景正常，可以正常定位
      */
     double lidarodom::checkLocalizability(std::vector<Eigen::Vector3d> planeNormals)
     {
          // 静态变量：标记是否永久处于退化状态
          static bool permanently_degenerate = false; 

          // 如果已经被标记为永久退化，直接返回退化状态
          if (permanently_degenerate)
          {
               return 1;
          }
          
          // 用于存储法向量矩阵的变量
          Eigen::MatrixXd mat;
          // 静态变量：连续退化帧计数器（暂未使用）
          static int stable_degenerate_count = 0;
          // 静态变量：连续退出退化帧计数器（暂未使用）
          static int stable_exit_degenerate_count = 0; 
          
          // 只有当平面法向量数量足够多（>10）时才进行退化检测
          if (planeNormals.size() > 10)
          {
               // 初始化矩阵，行数为法向量个数，列数为3（x,y,z分量）
               mat.setZero(planeNormals.size(), 3);
               
               // 将所有法向量填入矩阵，每行代表一个法向量的三个分量
               for (int i = 0; i < planeNormals.size(); i++)
               {
                    mat(i, 0) = planeNormals[i].x();  // x分量
                    mat(i, 1) = planeNormals[i].y();  // y分量
                    mat(i, 2) = planeNormals[i].z();  // z分量
               }
               
               // 对法向量矩阵进行奇异值分解（SVD），用于分析法向量的分布特性
               Eigen::JacobiSVD<Eigen::MatrixXd> svd(planeNormals.size(), 3);
               svd.compute(mat);

               // 计算退化指数：衡量最大和最小奇异值的差异程度
               double degeneracyIndex = (svd.singularValues().x() - svd.singularValues().z()) / svd.singularValues().y();
               // 计算稀疏指数：三个奇异值的平均值，反映整体的信息量
               double SparseIndex = (svd.singularValues().x() + svd.singularValues().z() + svd.singularValues().y())/3;

               // 被注释掉的调试输出和其他退化检测条件
               // if (svd.singularValues().z() < 10)
               // if (degeneracyIndex > 0.9 ){
               //    std::cout << ANSI_COLOR_YELLOW << "Low convincing result -> singular values:"
               //                << svd.singularValues().x() << ", " << svd.singularValues().y() << ", "
               //                << svd.singularValues().z() << ANSI_COLOR_RESET << std::endl;  
               // }
               // if (SparseIndex < 10 || degeneracyIndex > 0.90){ //mid360 1.5 - 3.55  //0.45 

               // 退化判断条件：稀疏指数小于10或最小奇异值小于7时认为处于退化场景
               if (SparseIndex < 10 || svd.singularValues().z() < 7 ){         
               // if (SparseIndex < 10 ){
               // if (svd.singularValues().z() < 4){  //avia
               // if (SparseIndex < 10){   
                    // 被注释掉的调试输出和连续帧退化检测逻辑
                    // std::cout << ANSI_COLOR_YELLOW << "Low convincing result -> singular values:"
                    //           << svd.singularValues().x() << ", " << svd.singularValues().y() << ", "
                    //           << svd.singularValues().z() << ANSI_COLOR_RESET << std::endl;
                    // std::cout << "Degenerate scene detected. Using external odometry." << std::endl;
                    // std::cout << "\033[2K\rDegenerate scene detected. Using external odometry." << std::flush;
                    // stable_degenerate_count++;
                    // stable_exit_degenerate_count = 0;
                    // if (stable_degenerate_count > 2) { // 连续多帧满足条件才返回退化
                    //      return 1;
                    // }
                    // else return 0;
                    return 1;  // 检测到退化场景，返回1
               } else return 0;  // 场景正常，返回0
               // }
               // else if(has_entered_degenerate){

               //      stable_degenerate_count = 0; // 重置退化计数器
               //      stable_exit_degenerate_count++;

               //      if (stable_exit_degenerate_count > 2) // 连续多帧满足退出退化条件才返回退出退化
               //      {
               //         return 0;
               //      }
               //      else
               //      {
               //         return 1; // 仍然处于退化状态
               //      }
               // }
          }
          else  // 平面法向量数量不足的情况
          {
               // 输出警告信息：接收到的法向量数量过少
               std::cout << ANSI_COLOR_RED << "Too few normal vector received -> " << planeNormals.size() << ANSI_COLOR_RESET << std::endl;
               // 设置为永久退化状态
               permanently_degenerate = true;
               return 1;  // 返回退化状态
          }

          // return 0;  // 被注释掉的默认返回值
     }

     Neighborhood lidarodom::computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }

     void lidarodom::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                       std::vector<point3D> &keypoints, const cloudFrame *p_frame)
     {
          auto estimatePointNeighborhood = [&](const std::vector<Eigen::Vector3d,
               Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
               const Eigen::Vector3d                                &location,
               double                                               &planarity_weight)
           {
             auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
             planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);
             // 保证法线方向一致
             if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
               neighborhood.normal = -neighborhood.normal;
             return neighborhood;
           };

         // 2) 权重归一化
         double lambda_weight       = std::abs(options_.weight_alpha);
         double lambda_neighborhood = std::abs(options_.weight_neighborhood);
         const double sum           = lambda_weight + lambda_neighborhood;
         lambda_weight /= sum;
         lambda_neighborhood /= sum;

         // 3) 一些阈值/邻域设置
         const short nb_voxels_visited = (p_frame->frame_id < options_.init_num_frames)
                                          ? 2
                                          : options_.voxel_neighborhood;
         const int kThresholdCapacity   = (p_frame->frame_id < options_.init_num_frames)
                                          ? 1
                                          : options_.threshold_voxel_occupancy;
         const double kMaxPointToPlane  = options_.max_dist_to_plane_icp;

         // 4) 预分配：估算最坏情况容量
         size_t max_possible =
           std::min<size_t>(keypoints.size() * options_.num_closest_neighbors,
                            options_.max_num_residuals);
         surf.reserve(max_possible);
         normals.reserve(max_possible);
         std::vector<point3D> valid_keypoints;
         valid_keypoints.reserve(max_possible);

         int num_residuals = 0;
         const size_t N    = keypoints.size();

         // 5) 主循环
         for (size_t k = 0; k < N; ++k)
         {
             const auto &kp        = keypoints[k];
             const auto &raw_point = kp.raw_point;

             // 5.1) 搜邻域
             std::vector<voxel> voxels;
             auto vector_neighbors = searchNeighbors(
               voxel_map,
               kp.point,
               nb_voxels_visited,
               options_.size_voxel_map,
               options_.max_number_neighbors,
               kThresholdCapacity,
               options_.estimate_normal_from_neighborhood ? nullptr : &voxels);

             if (vector_neighbors.size() < options_.min_number_neighbors)
                 continue;

             // 5.2) 计算初步权重
             double planarity_w = 0;
             Eigen::Vector3d location = TIL_ * raw_point;
             auto neighborhood = estimatePointNeighborhood(vector_neighbors,
                                                          location,
                                                          planarity_w);
             double weight = lambda_weight * planarity_w
                           + lambda_neighborhood
                             * std::exp(
                                 - (vector_neighbors[0] - kp.point).norm()
                                 / (kMaxPointToPlane * options_.min_number_neighbors)
                               );

             // 5.3) 对前 num_closest_neighbors 个邻域点都生成残差
             for (int i = 0;
                  i < options_.num_closest_neighbors
                  && size_t(i) < vector_neighbors.size();
                  ++i)
             {
                 double dist =
                   std::abs((kp.point - vector_neighbors[i]).transpose()
                            * neighborhood.normal);
                 if (dist >= options_.max_dist_to_plane_icp)
                     continue;

                 // 新增一个残差
                 ++num_residuals;

                 // 法向量归一化
                 Eigen::Vector3d nvec = neighborhood.normal.normalized();
                 double offset = -nvec.dot(vector_neighbors[i]);

                 // 创建 cost function
                 ceres::CostFunction *cost_f = nullptr;
                 switch (options_.icpmodel)
                 {
                   case IcpModel::CT_POINT_TO_PLANE:
                   {
         #ifdef USE_ANALYTICAL_DERIVATE
                     cost_f = new CT_ICP::CTLidarPlaneNormFactor(
                                raw_point,
                                nvec,
                                offset,
                                kp.alpha_time,
                                weight);
         #else
                     cost_f = CT_ICP::CTPointToPlaneFunctor::Create(
                                vector_neighbors[i],
                                raw_point,
                                nvec,
                                kp.alpha_time,
                                weight);
         #endif
                     break;
                   }
                   case IcpModel::POINT_TO_PLANE:
                   {
                     Eigen::Vector3d point_end =
                       p_frame->p_state->rotation.inverse()  * kp.point
                       - p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
         #ifdef USE_ANALYTICAL_DERIVATE
                     cost_f = new CT_ICP::LidarPlaneNormFactor(
                                point_end,
                                nvec,
                                offset,
                                weight);
         #else
                     cost_f = CT_ICP::PointToPlaneFunctor::Create(
                                vector_neighbors[i],
                                point_end,
                                nvec,
                                weight);
         #endif
                     break;
                   }
                 }

                 surf.push_back(cost_f);
                 normals.push_back(nvec);
                 valid_keypoints.push_back(kp);

                 if (num_residuals >= options_.max_num_residuals) break;
             }

             if (num_residuals >= options_.max_num_residuals) break;
         }

         // 6) 更新输出 keypoints & normals，使其与 surf 一一对应
         keypoints = std::move(valid_keypoints);
          
         
          {
//           auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
//                                                Eigen::Vector3d &location, double &planarity_weight)
//           {
//                auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
//                planarity_weight = std::pow(neighborhood.a2D, options_.power_planarity);

//                if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
//                {
//                     neighborhood.normal = -1.0 * neighborhood.normal;
//                }
//                return neighborhood;
//           };

//           double lambda_weight = std::abs(options_.weight_alpha);
//           double lambda_neighborhood = std::abs(options_.weight_neighborhood);
//           const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
//           const double sum = lambda_weight + lambda_neighborhood;

//           lambda_weight /= sum;
//           lambda_neighborhood /= sum;

//           const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames
//                                               ? 2
//                                               : options_.voxel_neighborhood;

//           const int kThresholdCapacity = p_frame->frame_id < options_.init_num_frames
//                                              ? 1
//                                              : options_.threshold_voxel_occupancy;

//           size_t num = keypoints.size();
//           int num_residuals = 0;

//           for (int k = 0; k < num; k++)
//           {
//                auto &keypoint = keypoints[k];
//                auto &raw_point = keypoint.raw_point;

//                std::vector<voxel> voxels;
//                auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
//                                                        nb_voxels_visited,
//                                                        options_.size_voxel_map,
//                                                        options_.max_number_neighbors,
//                                                        kThresholdCapacity,
//                                                        options_.estimate_normal_from_neighborhood
//                                                            ? nullptr
//                                                            : &voxels);

//                if (vector_neighbors.size() < options_.min_number_neighbors)
//                     continue;

//                double weight;

//                Eigen::Vector3d location = TIL_ * raw_point;

//                auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);

//                weight = lambda_weight * weight + lambda_neighborhood *
//                                                      std::exp(-(vector_neighbors[0] -
//                                                                 keypoint.point)
//                                                                    .norm() /
//                                                               (kMaxPointToPlane *
//                                                                options_.min_number_neighbors));

//                double point_to_plane_dist;
//                std::set<voxel> neighbor_voxels;
//                for (int i(0); i < options_.num_closest_neighbors; ++i)
//                {
//                     point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

//                     if (point_to_plane_dist < options_.max_dist_to_plane_icp)
//                     {

//                          num_residuals++;

//                          Eigen::Vector3d norm_vector = neighborhood.normal;
//                          norm_vector.normalize();

//                          normals.push_back(norm_vector); //   record normal

//                          double norm_offset = -norm_vector.dot(vector_neighbors[i]);

//                          switch (options_.icpmodel)
//                          {
//                          case IcpModel::CT_POINT_TO_PLANE:
//                          {
// #ifdef USE_ANALYTICAL_DERIVATE
//                               CT_ICP::CTLidarPlaneNormFactor *cost_function =
//                                   new CT_ICP::CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
// #else
//                               auto *cost_function = CT_ICP::CTPointToPlaneFunctor::Create(vector_neighbors[0],
//                                                                                           keypoints[k].raw_point,
//                                                                                           norm_vector,
//                                                                                           keypoints[k].alpha_time,
//                                                                                           weight);
// #endif
//                               surf.push_back(cost_function);
//                               // problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
//                               break;
//                          }
//                          case IcpModel::POINT_TO_PLANE:
//                          {
//                               Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * keypoints[k].point -
//                                                           p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
// #ifdef USE_ANALYTICAL_DERIVATE
//                               CT_ICP::LidarPlaneNormFactor *cost_function =
//                                   new CT_ICP::LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
// #else
//                               auto *cost_function = CT_ICP::PointToPlaneFunctor::Create(vector_neighbors[0],
//                                                                                         point_end, norm_vector, weight);
// #endif
//                               surf.push_back(cost_function);
//                               // problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());
//                               break;
//                          }
//                          }
//                     }
//                }

//                if (num_residuals >= options_.max_num_residuals)
//                     break;
//           }
     }
         
    }

     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };

     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     lidarodom::searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                                int nb_voxels_visited, double size_voxel_map,
                                int max_num_neighbors, int threshold_voxel_capacity,
                                std::vector<voxel> *voxels)
     {

          if (voxels != nullptr)
               voxels->reserve(max_num_neighbors);

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          priority_queue_t priority_queue;

          int max_iterations = 200; // 设置最大迭代次数
          int iteration_count = 0;

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         if (++iteration_count > max_iterations)
                         {
                            std::cerr << "Warning: Exceeded maximum iterations, exiting loop." << std::endl;
                            is_degenerate = true;
                            goto exit_loops; // 使用 goto 强行退出嵌套循环
                         }
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search != map.end())
                         {
                              const auto &voxel_block = search.value();
                              if (voxel_block.NumPoints() < threshold_voxel_capacity)
                                   continue;
                              for (int i(0); i < voxel_block.NumPoints(); ++i)
                              {
                                   auto &neighbor = voxel_block.points[i];
                                   double distance = (neighbor - point).norm();
                                   if (priority_queue.size() == max_num_neighbors)
                                   {
                                        if (distance < std::get<0>(priority_queue.top()))
                                        {
                                             priority_queue.pop();
                                             priority_queue.emplace(distance, neighbor, voxel_temp);
                                        }
                                   }
                                   else
                                        priority_queue.emplace(distance, neighbor, voxel_temp);
                              }
                         }
                    }
               }
          }

          exit_loops: // 跳转到这里退出循环

          auto size = priority_queue.size();
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
          if (voxels != nullptr)
          {
               voxels->resize(size);
          }
          for (auto i = 0; i < size; ++i)
          {
               closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
               if (voxels != nullptr)
                    (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
               priority_queue.pop();
          }

          return closest_neighbors;
     }

     void lidarodom::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                                   const double &intensity, double voxel_size,
                                   int max_num_points_in_voxel, double min_distance_points,
                                   int min_num_points, cloudFrame *p_frame)
     {
          short kx = static_cast<short>(point[0] / voxel_size);
          short ky = static_cast<short>(point[1] / voxel_size);
          short kz = static_cast<short>(point[2] / voxel_size);

          voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

          if (search != map.end())
          {
               auto &voxel_block = (search.value());

               if (!voxel_block.IsFull())
               {
                    double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                    for (int i(0); i < voxel_block.NumPoints(); ++i)
                    {
                         auto &_point = voxel_block.points[i];
                         double sq_dist = (_point - point).squaredNorm();
                         if (sq_dist < sq_dist_min_to_points)
                         {
                              sq_dist_min_to_points = sq_dist;
                         }
                    }
                    if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
                    {
                         if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                         {
                              voxel_block.AddPoint(point);
                              // addPointToPcl(points_world, point, intensity, p_frame);
                         }
                    }
               }
          }
          else
          {
               if (min_num_points <= 0)
               {
                    voxelBlock block(max_num_points_in_voxel);
                    block.AddPoint(point);
                    map[voxel(kx, ky, kz)] = std::move(block);
               }
          }
          addPointToPcl(points_world, point, intensity, p_frame);
     }

     void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d &point, const double &intensity, cloudFrame *p_frame)
     {
          pcl::PointXYZI cloudTemp;

          cloudTemp.x = point.x();
          cloudTemp.y = point.y();
          cloudTemp.z = point.z();
          cloudTemp.intensity = intensity;
          // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
          pcl_points->points.push_back(cloudTemp);
     }
     
     /*
     void lidarodom::map_incremental(cloudFrame *p_frame, int min_num_points)
     {
          //   only surf
          for (const auto &point : p_frame->point_surf)
          {
               addPointToMap(voxel_map, point.point, point.intensity,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points, p_frame);
          }

          {
               std::string laser_topic = "laser";
               pub_cloud_to_ros(laser_topic, points_world, p_frame->time_frame_end);
          }
          points_world->clear();
     }
     */

     void lidarodom::map_incremental(cloudFrame *p_frame, int min_num_points)
     {
          int count = 0;
          for (const auto &point : p_frame->point_surf)
          {
               addPointToMap(voxel_map, point.point, point.intensity,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points, p_frame);

               ++count;
               if (count % 2 == 0)
                    addPointToPcl(points_world, point.point, point.intensity, p_frame);

          }

          {
               if (!points_world->empty())
               {
                    std::string laser_topic = "laser";
                    pub_cloud_to_ros(laser_topic, points_world, p_frame->time_frame_end);
               }
               else
               {
                    LOG(INFO) << "No laser points provided.";
               }

               if (!p_frame->img_.empty())
               {
                    std::string img_topic = "camera";
                    pub_image_to_ros(img_topic, p_frame->img_, p_frame->time_frame_end );
               }
               else
               {
                    LOG(INFO) << "No image provided.";
               }
          }
          points_world->clear();
     }

     void lidarodom::lasermap_fov_segment()
     {
          //   use predict pose here
          Eigen::Vector3d location = current_state->translation;
          std::vector<voxel> voxels_to_erase;
          for (auto &pair : voxel_map)
          {
               Eigen::Vector3d pt = pair.second.points[0];
               if ((pt - location).squaredNorm() > (options_.max_distance * options_.max_distance))
               {
                    voxels_to_erase.push_back(pair.first);
               }
          }
          for (auto &vox : voxels_to_erase)
               voxel_map.erase(vox);
          std::vector<voxel>().swap(voxels_to_erase);
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                       double timestamp_begin, double timestamp_end)
     {
          std::vector<point3D> frame_surf(const_surf);
          if (index_frame < 2)
          {
               for (auto &point_temp : frame_surf)
               {
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }

          if (options_.motion_compensation == CONSTANT_VELOCITY)
               Undistort(frame_surf);

          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                              cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                              R_imu_lidar, t_imu_lidar);

          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          return p_frame;
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, cv::Mat& img,  state *cur_state,
                                            double timestamp_begin, double timestamp_end)
     {
          std::vector<point3D> frame_surf(const_surf);
          if (index_frame < 2)
          {
               for (auto &point_temp : frame_surf)
               {
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }

          if (options_.motion_compensation == CONSTANT_VELOCITY)
               Undistort(frame_surf);

          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                              cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                              R_imu_lidar, t_imu_lidar);

          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          p_frame->img_ = img;

          return p_frame;
     }
     
     /**
      * [功能描述]：初始化当前帧的状态，为后续的位姿估计提供初值
      * 该函数负责设置激光雷达帧开始和结束时刻的位姿初值
      * 对于前两帧使用外部里程计或IMU数据初始化，对于后续帧使用历史状态和IMU预测
      */
     void lidarodom::stateInitialization()
     {
          // 重力对齐矩阵计算（当前已注释，在构造函数中计算）
          // Eigen::Matrix3d R_align = computeGravityAlignment(g_odom, g_imu);

          // ==================== 系统初始化阶段（前两帧）====================
          // 只对前两帧进行特殊的初始化处理，建立系统的初始位姿基准
          if (index_frame < 2) // 仅处理第1帧和第2帧 
          {
               // ==================== 使用外部里程计数据初始化 ====================
               // 如果外部里程计队列不为空，优先使用外部里程计数据进行初始化
               if (!odomQueue.empty()){        
                    /// 注意：世界坐标系相同，但初始化得到的初始位姿可能不同
                    
                    // ==================== 查找帧开始时刻对应的里程计数据 ====================
                    // 检查队列首个数据的时间戳是否晚于帧开始时间
                    if(!odomQueue.front().header.stamp.toSec() > time_begin){
                         // 使用二分查找找到第一个时间戳不小于帧开始时间的里程计数据
                         auto it = std::lower_bound(
                                        odomQueue.begin(), odomQueue.end(), time_begin,
                                        [](const nav_msgs::Odometry &odom, double time) {
                                        return odom.header.stamp.toSec() < time;});
                    
                         // 如果找到了有效的里程计数据
                         if (it != odomQueue.end()) {
                              startOdomMsg = *it;  // 保存开始时刻的里程计消息
                    
                              // ==================== 四元数转换和姿态提取 ====================
                              // 将ROS四元数转换为tf四元数格式
                              tf::Quaternion orientation;
                              tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);
                              
                              // 提取欧拉角（当前代码中声明但未使用）
                              double roll, pitch, yaw;
                              tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

                              // 转换为Eigen四元数和位置向量
                              Eigen::Quaterniond odom_quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
                              Eigen::Vector3d odom_trans(
                                                  startOdomMsg.pose.pose.position.x,
                                                  startOdomMsg.pose.pose.position.y,
                                                  startOdomMsg.pose.pose.position.z);
                              
                              // ==================== 应用重力对齐变换 ====================
                              // 将外部里程计的姿态和位置转换到当前坐标系
                              current_state->rotation_begin = Eigen::Quaterniond(R_align * odom_quat.toRotationMatrix());
                              current_state->translation_begin = R_align * odom_trans;
                         }
                    }
                    
                    // ==================== 查找帧结束时刻对应的里程计数据 ====================
                    // 检查队列末尾数据的时间戳是否早于帧结束时间
                    if (!odomQueue.back().header.stamp.toSec() < time_curr){
                         // 使用二分查找找到第一个时间戳不小于帧结束时间的里程计数据
                         auto it = std::lower_bound(odomQueue.begin(), odomQueue.end(), time_curr,
                         [](const nav_msgs::Odometry &odom, double time) {
                              return odom.header.stamp.toSec() < time;
                         });

                         // 如果找到了有效的里程计数据
                         if (it != odomQueue.end()) {
                         endOdomMsg = *it;  // 保存结束时刻的里程计消息

                         // ==================== 姿态和位置提取 ====================
                         // 将ROS四元数转换为tf四元数格式
                         tf::Quaternion orientation;
                         tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
                         
                         // 转换为Eigen四元数和位置向量
                         Eigen::Quaterniond odom_quat(orientation.w(), orientation.x(), orientation.y(), orientation.z());
                         Eigen::Vector3d odom_trans(
                                   endOdomMsg.pose.pose.position.x,
                                   endOdomMsg.pose.pose.position.y,
                                   endOdomMsg.pose.pose.position.z);
                         
                         // ==================== 应用重力对齐变换 ====================
                         // 将外部里程计的姿态和位置转换到当前坐标系
                         current_state->rotation = Eigen::Quaterniond(R_align * odom_quat.toRotationMatrix());
                         current_state->translation = R_align * odom_trans;
                         }
                    }
                    
                    // ==================== 协方差一致性检查 ====================
                    // 通过比较协方差矩阵的第一个元素来判断开始和结束里程计数据的一致性
                    // 如果协方差不一致，说明数据来源不同，改用IMU数据初始化
                    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0]))){
                         // 使用IMU状态序列的首尾状态进行初始化
                         current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
                         current_state->translation_begin = imu_states_.front().p_;
                         current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
                         current_state->translation = imu_states_.back().p_; 
                    }
               }
               // ==================== 使用IMU数据初始化（备选方案）====================
               // 如果外部里程计队列为空，则使用IMU预测的状态进行初始化
               else{
                         // 帧开始状态：使用IMU状态序列的第一个状态
                         current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
                         current_state->translation_begin = imu_states_.front().p_;
                         
                         // 帧结束状态：使用IMU状态序列的最后一个状态
                         current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
                         current_state->translation = imu_states_.back().p_;
               }
          }
          // ==================== 正常运行阶段（第三帧及以后）====================
          // 对于后续帧，使用历史状态信息和IMU预测进行初始化
          else
          {
               // ==================== 使用上一帧的结束状态 ====================
               // 将上一帧的结束位姿作为当前帧的开始位姿
               // 这确保了帧间的连续性和一致性
               current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
               current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
               
               // 备选方案：使用点云帧历史（注释掉，因为内存消耗大）
               // current_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
               // current_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
               
               // ==================== 使用IMU预测的状态 ====================
               // 将IMU预测的最终状态作为当前帧的结束位姿初值
               // 这为后续的优化过程提供了良好的初始估计
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
               
               // 备选变量（注释掉）
               // current_state->rotation = q_next_end;
               // current_state->translation = t_next_end;
          }
     }

     std::vector<MeasureGroup> lidarodom::getMeasureMents()
     {
          std::vector<MeasureGroup> measurements;
          while (true)
          {
               // if (imu_buffer_.empty())
               //      return measurements;

               // if (lidar_buffer_.empty())
               //      return measurements;

               // if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
               //      return measurements;
               
               if (imu_buffer_.empty() || lidar_buffer_.empty() || img_buffer_.empty())
                    return measurements;

               // 最新的imu数据需要覆盖掉一帧lidar数据 | time_curr为上一次激光雷达帧的结束时间
               if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
                    return measurements;

               if (imu_buffer_.back()->timestamp_ < img_time_buffer_.front())
                    return measurements;
               
               // 获取之前还未使用的lidar点
               auto curr_lidar = lidar_buffer_.front();
               auto lidar = last_lidar_;
               lidar.reserve(lidar.size() + curr_lidar.size());  // 避免后期编译器自己分配内存, 现在手动分配内存
               lidar.insert(lidar.end(), curr_lidar.begin(), curr_lidar.end());
               lidar_buffer_.pop_front();
               time_buffer_.pop_front();
               double begin_time = lidar.front().timestamp;
               double end_time = lidar.back().timestamp;

               if (!odomQueue.empty() && odomQueue.front().header.stamp.toSec() < time_begin - delay_time_)
                    odomQueue.pop_front();

               // 筛选相机帧
               std::vector<double> times;
               std::vector<cv::Mat> images;
               int count = 0;
               while (!img_time_buffer_.empty() && !img_buffer_.empty() && img_time_buffer_.front() <= end_time)
               {
                    ++count;
                    if (count > 2 && img_time_buffer_.front() >= begin_time)
                    {
                         images.emplace_back(img_buffer_.front());
                         times.emplace_back(img_time_buffer_.front());
                    }
                    img_buffer_.pop_front();
                    img_time_buffer_.pop_front();
               }
               // 避免无images的情况
               if (times.empty())
               {
                    last_lidar_ = lidar;
                    return measurements;
               }

               // 筛选雷达数据
               std::vector<std::vector<point3D>> points;
               std::vector<std::deque<IMUPtr>> imus;

               // 直接按照时间切割
               size_t p = 0; // lidar点指针
               // double left = begin_time;
               for (size_t k = 0; k < times.size(); ++k)
               {
                    double right = times[k];
                    // 分割Lidar点
                    size_t p_left = p;
                    while (p < lidar.size() && lidar[p].timestamp <= right)
                         ++p;
                    points.emplace_back(lidar.begin() + p_left, lidar.begin() + p);
                    // left = right;
               }

               // 未使用的剩余点则进行保留
               last_lidar_.clear();
               last_lidar_.insert(last_lidar_.end(), lidar.begin() + p, lidar.end());

               // 筛选imu帧
               double prev_time = begin_time;
               for (size_t k = 0; k < times.size(); ++k) {
                    double curr_time = times[k];
                    std::deque<IMUPtr> curr_imu;
                    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp_ <= curr_time)
                    {
                         if (imu_buffer_.front()->timestamp_ > prev_time)
                         {
                              curr_imu.push_back(imu_buffer_.front());
                         }
                         imu_buffer_.pop_front();
                    }
                    imus.emplace_back(curr_imu);
                    prev_time = curr_time;
               }

               // 整合测量数据
               for (size_t k = 0; k < times.size(); ++k)
               {
                    // 这里meas对应的时间在原始的lidar的基础上进行了修改
                    MeasureGroup meas;
                    meas.lidar_ = points[k];
                    meas.lidar_begin_time_ = k == 0 ? begin_time : times[k-1];
                    meas.lidar_end_time_ = times[k];
                    meas.imu_ = imus[k];
                    meas.img_ = images[k];
                    measurements.emplace_back(meas);
               }
               time_curr = end_time;
     
 
               {
               // MeasureGroup meas;

               // meas.lidar_ = lidar_buffer_.front();
               // meas.lidar_begin_time_ = time_buffer_.front().first;
               // meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
               // lidar_buffer_.pop_front();
               // time_buffer_.pop_front();

               // time_curr = meas.lidar_end_time_;
               // time_begin = meas.lidar_begin_time_;

               // if (!odomQueue.empty() && odomQueue.front().header.stamp.toSec() < time_begin - delay_time_)
               //      odomQueue.pop_front();

               // double imu_time = imu_buffer_.front()->timestamp_;
               // meas.imu_.clear();
               // while ((!imu_buffer_.empty()) && (imu_time < meas.lidar_end_time_))
               // {
               //      imu_time = imu_buffer_.front()->timestamp_;
               //      if (imu_time > meas.lidar_end_time_)
               //      {
               //           break;
               //      }
               //      meas.imu_.push_back(imu_buffer_.front());
               //      imu_buffer_.pop_front();
               // }

               // if (!imu_buffer_.empty())
               //      meas.imu_.push_back(imu_buffer_.front()); //   added for Interp

               // measurements.push_back(meas);
               }
          
          }
     }

     /**
      * [功能描述]：使用IMU数据进行状态预测的核心函数
      * 该函数基于ESKF（误差状态卡尔曼滤波器）对系统状态进行预测更新
      * 通过处理IMU测量数据序列，推算激光雷达帧结束时刻的精确状态
      * 对于超出目标时间的IMU数据，采用线性插值方法获取精确时刻的IMU值
      */
     void lidarodom::Predict()
     {
          // 将当前ESKF的名义状态添加到状态历史序列中
          // 这个状态作为预测的起始点，对应激光雷达帧开始时刻的状态
          imu_states_.emplace_back(eskf_.GetNominalState());

          // ==================== 状态预测初始化 ====================
          /// 获取目标预测时间（激光雷达帧结束时刻）
          double time_current = measures_.lidar_end_time_;
          
          // 声明临时变量用于存储上一时刻的角速度和加速度（当前代码中未使用）
          Vec3d last_gyr, last_acc;
          
          // ==================== IMU数据序列处理 ====================
          // 遍历当前测量组中的所有IMU数据，按时间顺序进行状态预测
          for (auto &imu : measures_.imu_)
          {
               // 获取当前IMU数据的时间戳
               double time_imu = imu->timestamp_;
               
               // ==================== 情况1：IMU时间在目标时间之前或等于目标时间 ====================
               // 对于时间戳不超过激光雷达帧结束时间的IMU数据，直接用于状态预测
               if (imu->timestamp_ <= time_current)
               {
                    // 初始化：如果这是第一个IMU数据，将其设置为上一个IMU数据
                    if (last_imu_ == nullptr)
                         last_imu_ = imu;
                    
                    // 使用当前IMU数据更新ESKF状态预测
                    // 这会根据IMU的角速度和加速度测量值推进系统状态
                    eskf_.Predict(*imu);
                    
                    // 将预测后的状态添加到状态历史序列中
                    // 用于后续的运动补偿和插值操作
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    
                    // 更新上一个IMU数据的记录，用于后续插值计算
                    last_imu_ = imu;
               }
               // ==================== 情况2：IMU时间超过目标时间 ====================
               // 对于时间戳超过激光雷达帧结束时间的IMU数据，需要进行时间插值
               else
               {
                    // ==================== 线性插值权重计算 ====================
                    // 计算当前IMU时间与目标时间的间隔
                    double dt_1 = time_imu - time_current;
                    
                    // 计算目标时间与上一个IMU时间的间隔
                    double dt_2 = time_current - last_imu_->timestamp_;
                    
                    // 计算线性插值的权重系数
                    // w1对应上一个IMU数据的权重，w2对应当前IMU数据的权重
                    // 距离目标时间越近的数据权重越大
                    double w1 = dt_1 / (dt_1 + dt_2);  // 上一个IMU数据的权重
                    double w2 = dt_2 / (dt_1 + dt_2);  // 当前IMU数据的权重
                    
                    // ==================== IMU数据插值 ====================
                    // 对加速度进行线性插值，获得目标时刻的精确加速度值
                    Eigen::Vector3d acc_temp = w1 * last_imu_->acce_ + w2 * imu->acce_;
                    
                    // 对角速度进行线性插值，获得目标时刻的精确角速度值
                    Eigen::Vector3d gyr_temp = w1 * last_imu_->gyro_ + w2 * imu->gyro_;
                    
                    // ==================== 创建插值IMU数据 ====================
                    // 使用插值得到的IMU数据创建新的IMU对象
                    // 时间戳设置为目标时间（激光雷达帧结束时间）
                    IMUPtr imu_temp = std::make_shared<zjloc::IMU>(time_current, gyr_temp, acc_temp);
                    
                    // 使用插值后的IMU数据进行最终的状态预测
                    // 这确保了状态预测刚好到达激光雷达帧结束时刻
                    eskf_.Predict(*imu_temp);
                    
                    // 将最终预测状态添加到状态历史序列中
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    
                    // 更新上一个IMU数据记录为插值后的数据
                    // 这对于下一帧的处理很重要
                    last_imu_ = imu_temp;
               }
          }
     }

     void lidarodom::Undistort(std::vector<point3D> &points)
     {
          // auto &cloud = measures_.lidar_;
          auto imu_state = eskf_.GetNominalState(); // 最后时刻的状态
          // std::cout << __FUNCTION__ << ", " << imu_state.timestamp_ << std::endl;
          SE3 T_end = SE3(imu_state.R_, imu_state.p_);

          /// 将所有点转到最后时刻状态上
          for (auto &pt : points)
          {
               SE3 Ti = T_end;
               NavStated match;

               // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
               math::PoseInterp<NavStated>(
                   pt.timestamp, imu_states_, [](const NavStated &s)
                   { return s.timestamp_; },
                   [](const NavStated &s)
                   { return s.GetSE3(); },
                   Ti, match);

               pt.raw_point = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt.raw_point;
          }
     }

     void lidarodom::TryInitIMU()
     {
          for (auto imu : measures_.imu_)
          {
               imu_init_.AddIMU(*imu);
          }

          if (imu_init_.InitSuccess())
          {
               // 读取初始零偏，设置ESKF
               zjloc::ESKFD::Options options;
               // 噪声由初始化器估计
               // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
               // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
               // options.update_bias_acce_ = false;
               // options.update_bias_gyro_ = false;
               eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
               imu_need_init_ = false;

               std::cout << ANSI_COLOR_GREEN_BOLD << "IMU初始化成功" << ANSI_COLOR_RESET << std::endl;
          }
     }

     
}
