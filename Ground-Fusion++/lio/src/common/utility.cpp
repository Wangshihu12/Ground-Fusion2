#include "utility.h"

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb)
{
     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb)
{
     Eigen::Quaterniond q_a = Eigen::Quaterniond(Sophus::SO3d::exp(qa).matrix());
     Eigen::Quaterniond q_b = Eigen::Quaterniond(Sophus::SO3d::exp(qb).matrix());
     q_a.normalize(), q_b.normalize();

     Eigen::Matrix3d rota = q_a.toRotationMatrix();
     Eigen::Matrix3d rotb = q_b.toRotationMatrix();

     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b)
{
     Eigen::Matrix3d rota = q_a.toRotationMatrix();
     Eigen::Matrix3d rotb = q_b.toRotationMatrix();

     double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
     norm = std::acos(norm) * 180 / M_PI;
     return norm;
}

void subSampleFrame(std::vector<point3D> &frame, double size_voxel)
{
     std::tr1::unordered_map<voxel, std::vector<point3D>, std::hash<voxel>> grid;
     for (int i = 0; i < (int)frame.size(); i++)
     {
          auto kx = static_cast<short>(frame[i].point[0] / size_voxel);
          auto ky = static_cast<short>(frame[i].point[1] / size_voxel);
          auto kz = static_cast<short>(frame[i].point[2] / size_voxel);
          grid[voxel(kx, ky, kz)].push_back(frame[i]);
     }
     frame.resize(0);
     int step = 0;
     for (const auto &n : grid)
     {
          if (n.second.size() > 0)
          {
               frame.push_back(n.second[0]);
               step++;
          }
     }
}

/**
 * [功能描述]：对点云进行网格采样，通过体素化方法降低点云密度并提取关键点
 * 该函数使用体素网格对输入点云进行下采样，在保持点云几何特征的同时减少数据量
 * 主要用于激光雷达里程计中的特征点提取，提高后续配准算法的计算效率
 * 
 * @param frame：输入的原始点云数据，包含所有待采样的3D点
 * @param keypoints：输出的关键点容器，存储采样后的稀疏点云
 * @param size_voxel_subsampling：体素网格的尺寸大小，决定采样的稀疏程度
 *                                值越大采样越稀疏，值越小保留的点越多
 */
void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
    // ==================== 输出容器初始化 ====================
    // 清空关键点容器，确保输出结果不受之前数据影响
    keypoints.resize(0);
    
    // ==================== 创建点云副本 ====================
    // 创建输入点云的完整副本，避免修改原始数据
    // 这种做法确保了函数的数据安全性，原始点云保持不变
    std::vector<point3D> frame_sub;
    frame_sub.resize(frame.size());  // 预分配内存空间，提高效率
    
    // 逐点复制原始点云数据到副本中
    for (int i = 0; i < (int)frame_sub.size(); i++)
    {
        frame_sub[i] = frame[i];  // 深拷贝每个点的完整信息
    }
    
    // ==================== 执行体素化采样 ====================
    // 调用核心采样函数对点云副本进行体素网格下采样
    // subSampleFrame函数会就地修改frame_sub，移除冗余点，保留代表性点
    // 采样策略通常是在每个体素内选择一个或几个最具代表性的点
    subSampleFrame(frame_sub, size_voxel_subsampling);
    
    // ==================== 结果输出准备 ====================
    // 为关键点容器预分配内存，避免频繁内存重分配
    // 采样后的点数量通常远小于原始点数量
    keypoints.reserve(frame_sub.size());
    
    // ==================== 采样结果转移 ====================
    // 将采样后的所有点转移到输出容器中
    for (int i = 0; i < (int)frame_sub.size(); i++)
    {
        keypoints.push_back(frame_sub[i]);  // 保留采样后点的所有属性信息
    }
    
    // 函数执行完毕后，keypoints包含了原始点云的稀疏采样结果
    // 这些关键点在保持原始几何结构的同时，大幅减少了数据量
    // 为后续的点云配准、特征匹配等算法提供高效的输入数据
}

void distortFrame(std::vector<point3D> &points, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end,
                  Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
     Eigen::Quaterniond q_end_inv = q_end.inverse();         // Rotation of the inverse pose
     Eigen::Vector3d t_end_inv = -1.0 * (q_end_inv * t_end); // Translation of the inverse pose
     for (auto &point_temp : points)
     {
          double alpha_time = point_temp.alpha_time;
          Eigen::Quaterniond q_alpha = q_begin.slerp(alpha_time, q_end);
          q_alpha.normalize();
          Eigen::Vector3d t_alpha = (1.0 - alpha_time) * t_begin + alpha_time * t_end;

          point_temp.raw_point = R_imu_lidar.transpose() *
                                     (q_end_inv * (q_alpha * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t_alpha) + t_end_inv) -
                                 R_imu_lidar.transpose() * t_imu_lidar;
     }
}

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end,
                    Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end, Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
     Eigen::Vector3d t;
     Eigen::Matrix3d R;
     double alpha_time = point_temp.alpha_time;
     switch (compensation)
     {
     case MotionCompensation::NONE:
     case MotionCompensation::CONSTANT_VELOCITY:
          R = q_end.toRotationMatrix();
          t = t_end;
          break;
     case MotionCompensation::CONTINUOUS:
     case MotionCompensation::ITERATIVE:
          R = q_begin.slerp(alpha_time, q_end).normalized().toRotationMatrix();
          t = (1.0 - alpha_time) * t_begin + alpha_time * t_end;
          break;
     }
     point_temp.point = R * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t;
}