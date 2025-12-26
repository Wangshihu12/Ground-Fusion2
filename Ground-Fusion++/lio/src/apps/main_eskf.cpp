// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>
#include <std_msgs/String.h> 

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert.h"
#include "liw/lio/lidarodom.h"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

nav_msgs::Path laserOdoPath;
nav_msgs::Path orinOdoPath;
nav_msgs::Path orinvinsOdoPath;

zjloc::lidarodom *lio;
zjloc::CloudConvert *convert;
std::mutex odoLock; //vins里程计锁

DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{

    std::vector<point3D> cloud_out;
    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    zjloc::common::Timer::Evaluate([&]()
                                   { 
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                   "laser ds");
    
    zjloc::common::Timer::Evaluate([&]()
    {
        std::sort(cloud_out.begin(), cloud_out.end(), [](const point3D &a, const point3D &b) -> bool
        {
            return a.relative_time < b.relative_time;
        });
    }, "laser sort");

    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan()));
}

void standard_pcl_cbk2(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    static int c = 0;
    // if (c % 2 == 0 && use_velodyne)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            double sample_size = lio->getIndex() < 20 ? 0.01 : 0.05;
            // double sample_size = 0.05;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        },
                                       "laser ds");

        zjloc::common::Timer::Evaluate([&]()
        {
            std::sort(cloud_out.begin(), cloud_out.end(), [](const point3D &a, const point3D &b) -> bool
            {
                return a.relative_time < b.relative_time;
            });
        }, "laser sort");

        // lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec() - convert->getTimeSpan(), convert->getTimeSpan())); //  FIXME: for staircase dataset(header timestamp is the frame end)
        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), convert->getTimeSpan())); //  normal
    }
    c++;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
{
    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.toSec(),
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
    lio->pushData(imu);
}

void compressed_image_cbk(const sensor_msgs::CompressedImageConstPtr &msg)
{
    // note 跟之前的voxelmap的使用方法不同, 直接调用cv_bridge出现问题
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    lio->pushData(img, msg->header.stamp.toSec());
    img.release();
}

void updateStatus(const std_msgs::Int32::ConstPtr &msg)
{
    int type = msg->data;
    if (type == 1)
    {
    }
    else if (type == 2)
    {
    }
    else if (type == 3)
        ;
    else if (type == 4)
        ;
    else
        ;
}

/**
 * [功能描述]：激光雷达里程计主程序入口，基于ESKF算法的SLAM系统
 * @param argc：命令行参数个数
 * @param argv：命令行参数数组
 * @return 程序退出状态码，0表示正常退出，-1表示初始化失败
 */
int main(int argc, char **argv)
{
    // 初始化Google日志系统
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;     // 设置标准错误输出的日志级别为INFO
    FLAGS_colorlogtostderr = true;           // 启用彩色日志输出
    google::ParseCommandLineFlags(&argc, &argv, true);  // 解析命令行参数

    // 初始化ROS节点
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // 从ROS参数服务器获取配置文件路径
    std::string config_file;
    if (nh.getParam("config_file", config_file))
    {
        // 成功获取配置文件路径，输出绿色提示信息
        std::cout << "\033[32m" << "config_file: " << config_file << "\033[0m" << std::endl;
    }
    else
    {
        // 获取配置文件路径失败，输出错误信息
        ROS_ERROR("Failed to get param 'config_file'");
    }

    // std::string config_file = std::string(ROOT_DIR) + "config/mapping.yaml";  // 被注释的硬编码配置文件路径
    // std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    // 创建激光雷达里程计对象并初始化
    lio = new zjloc::lidarodom();
    if (!lio->init(config_file))
    {
        return -1;  // 初始化失败，退出程序
    }

    // 创建点云数据发布器
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::PointCloud2>("scan", 10);
    
    // 定义点云发布回调函数，用于将处理后的点云数据发布到ROS话题
    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time)
        {
            // 创建ROS点云消息指针
            sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
            // 将PCL点云转换为ROS点云消息格式
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            // 设置消息头信息
            cloud_ptr_output->header.stamp = ros::Time().fromSec(time);  // 设置时间戳
            cloud_ptr_output->header.frame_id = "map";                   // 设置坐标系为map
            
            // 根据话题名称决定是否发布点云数据
            if (topic_name == "laser")
                pub_scan.publish(*cloud_ptr_output);  // 发布激光雷达点云
            else
                ; // publisher_.publish(*cloud_ptr_output);  // 其他点云发布器（被注释）
            return true;
        }
    );

    // 创建各种里程计和路径相关的发布器
    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);                    // 激光里程计发布器
    ros::Publisher pubLaserOdometryPath = nh.advertise<nav_msgs::Path>("/odometry_path", 5);             // 激光里程计路径发布器
    ros::Publisher puborinOdometryPath = nh.advertise<nav_msgs::Path>("/orin_odometry_path", 5);         // Orin设备里程计路径发布器
    ros::Publisher puborinvinsPath = nh.advertise<nav_msgs::Path>("/orin_vins_path", 5);                // Orin设备VINS路径发布器
    ros::Publisher pubLaserPose = nh.advertise<geometry_msgs::PoseStamped>("/laser_pose", 100);         // 激光位姿发布器
    ros::Publisher puborinLaserPose = nh.advertise<geometry_msgs::PoseStamped>("/orin_laser_pose", 100); // Orin设备激光位姿发布器

    // 定义位姿发布回调函数，用于发布不同类型的位姿和里程计数据
    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, double stamp)
        {
            // 静态TF广播器，用于发布坐标变换
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            
            // 从SE3位姿中提取四元数和平移向量
            Eigen::Quaterniond q_current(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            
            // 处理激光雷达位姿数据
            if (topic_name == "laser")
            {
                // 广播从map到base_link的坐标变换
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "base_link"));

                // 发布激光里程计消息
                nav_msgs::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = ros::Time().fromSec(stamp);

                // 设置位姿信息（四元数和位置）
                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();
                pubLaserOdometry.publish(laserOdometry);

                // 发布激光位姿和路径信息
                geometry_msgs::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                pubLaserPose.publish(laserPose);
                
                // 更新并发布路径信息
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "/map";
                pubLaserOdometryPath.publish(laserOdoPath);
            }

            // 处理Orin设备激光雷达位姿数据
            if (topic_name == "orin_laser")
            {
                // 广播从map到orin_base_link的坐标变换
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "orin_base_link"));

                // 创建Orin设备里程计消息
                nav_msgs::Odometry orinOdometry;
                orinOdometry.header.frame_id = "map";
                orinOdometry.child_frame_id = "orin_base_link";
                orinOdometry.header.stamp = ros::Time().fromSec(stamp);

                // 设置Orin设备位姿信息
                orinOdometry.pose.pose.orientation.x = q_current.x();
                orinOdometry.pose.pose.orientation.y = q_current.y();
                orinOdometry.pose.pose.orientation.z = q_current.z();
                orinOdometry.pose.pose.orientation.w = q_current.w();
                orinOdometry.pose.pose.position.x = pose.translation().x();
                orinOdometry.pose.pose.position.y = pose.translation().y();
                orinOdometry.pose.pose.position.z = pose.translation().z();
                // pubLaserOdometry.publish(laserOdometry);  // 被注释的发布语句

                // 发布Orin设备位姿和路径信息
                geometry_msgs::PoseStamped orinPose;
                orinPose.header = orinOdometry.header;
                orinPose.pose = orinOdometry.pose.pose;
                puborinLaserPose.publish(orinPose);
                
                // 更新并发布Orin设备路径信息
                orinOdoPath.header.stamp = orinOdometry.header.stamp;
                orinOdoPath.poses.push_back(orinPose);
                orinOdoPath.header.frame_id = "/map";
                puborinOdometryPath.publish(orinOdoPath);
            }

            // 处理Orin设备VINS位姿数据
            if (topic_name == "orin_vins")
            {
                // 广播从map到orin_vins的坐标变换
                br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(stamp), "map", "orin_vins"));

                // 创建Orin VINS里程计消息
                nav_msgs::Odometry orinvinsOdometry;
                orinvinsOdometry.header.frame_id = "map";
                orinvinsOdometry.child_frame_id = "orin_vins";
                orinvinsOdometry.header.stamp = ros::Time().fromSec(stamp);

                // 设置Orin VINS位姿信息
                orinvinsOdometry.pose.pose.orientation.x = q_current.x();
                orinvinsOdometry.pose.pose.orientation.y = q_current.y();
                orinvinsOdometry.pose.pose.orientation.z = q_current.z();
                orinvinsOdometry.pose.pose.orientation.w = q_current.w();
                orinvinsOdometry.pose.pose.position.x = pose.translation().x();
                orinvinsOdometry.pose.pose.position.y = pose.translation().y();
                orinvinsOdometry.pose.pose.position.z = pose.translation().z();
                // pubLaserOdometry.publish(laserOdometry);  // 被注释的发布语句

                // 发布Orin VINS路径信息
                geometry_msgs::PoseStamped orinvinsPose;
                orinvinsPose.header = orinvinsOdometry.header;
                orinvinsPose.pose = orinvinsOdometry.pose.pose;
                
                // 更新并发布Orin VINS路径信息
                orinvinsOdoPath.header.stamp = orinvinsOdometry.header.stamp;
                orinvinsOdoPath.poses.push_back(orinvinsPose);
                orinvinsOdoPath.header.frame_id = "/map";
                puborinvinsPath.publish(orinvinsOdoPath);
            }

            return true;
        }
    );

    // 创建图像发布器
    ros::Publisher image_pub = nh.advertise<sensor_msgs::CompressedImage>("/img", 10);
    
    // 定义图像发布回调函数，用于发布压缩图像数据
    auto image_pub_func = std::function<bool(std::string& topic_name, cv::Mat image, double time)>(
        [&](std::string& topic_name, cv::Mat image, double time)
        {
            // 1. 构造ROS消息头
            std_msgs::Header header;
            header.stamp = ros::Time().fromSec(time);
            header.frame_id = "camera";

            // 2. 将cv::Mat图像压缩为JPEG格式
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};  // 设置JPEG压缩质量为90%
            cv::imencode(".jpg", image, buffer, params);

            // 3. 填充压缩图像消息并发布
            sensor_msgs::CompressedImagePtr msg(new sensor_msgs::CompressedImage());
            msg->header = header;
            msg->format = "jpeg";
            msg->data   = std::move(buffer);

            image_pub.publish(msg);
            return true;
        }
    );

    // 创建数据发布器（速度、距离、文本信息）
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("/velocity", 1);    // 速度发布器
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/move_dist", 1);  // 移动距离发布器
    ros::Publisher text_pub = nh.advertise<std_msgs::String>("/text", 1);       // 文本信息发布器

    // 定义数据发布回调函数，用于发布速度、距离和文本信息
    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, std::string time2)>(
        [&](std::string &topic_name, double time1, std::string time2)
        {
            std_msgs::Float32 time_rviz;  // 用于RViz显示的浮点数消息
            std_msgs::String text_msg;    // 文本消息

            text_msg.data = time2;        // 设置文本内容
            time_rviz.data = time1;       // 设置数值数据
            
            // 根据话题名称发布相应的数据
            if (topic_name == "velocity")
                vel_pub.publish(time_rviz);   // 发布速度信息
            else
                dist_pub.publish(time_rviz);  // 发布距离信息

            if (topic_name == "text") 
                text_pub.publish(text_msg);   // 发布文本信息

            return true;
        }
    );

    // 将所有回调函数注册到激光雷达里程计对象中
    lio->setFunc(cloud_pub_func);  // 设置点云发布函数
    lio->setFunc(pose_pub_func);   // 设置位姿发布函数
    lio->setFunc(data_pub_func);   // 设置数据发布函数
    lio->setFunc(image_pub_func);  // 设置图像发布函数

    // 创建并初始化点云转换对象
    convert = new zjloc::CloudConvert;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    // 从配置文件中读取话题名称
    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();  // 激光雷达话题
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();    // IMU话题
    std::string image_topic = yaml["common"]["img_topic"].as<std::string>();  // 图像话题

    // 根据激光雷达类型创建相应的订阅器
    ros::Subscriber subLaserCloud = convert->lidar_type_ == zjloc::CloudConvert::LidarType::AVIA
                                        ? nh.subscribe(laser_topic, 100, livox_pcl_cbk)        // Livox AVIA激光雷达
                                        : nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 100, standard_pcl_cbk2);  // 标准点云格式

    // 订阅IMU数据
    ros::Subscriber sub_imu_ori = nh.subscribe<sensor_msgs::Imu>(imu_topic, 500, imuHandler);

    // 订阅状态切换命令
    ros::Subscriber sub_type = nh.subscribe<std_msgs::Int32>("/change_status", 2, updateStatus);

    // 订阅VINS里程计数据，使用Lambda表达式作为回调函数
    ros::Subscriber subOdom = nh.subscribe<nav_msgs::Odometry>(
                              "/vins/odometry/imu_propagate_ros", 
                              2000, 
                              [lio](const nav_msgs::Odometry::ConstPtr& msg){
                                lio->pushData(msg);  // 将里程计数据推送给激光雷达里程计处理器
                              }, 
                              ros::TransportHints().tcpNoDelay());  // 设置TCP无延迟传输

    // 订阅压缩图像数据
    ros::Subscriber sub_image = nh.subscribe( image_topic, 200, compressed_image_cbk);

    // 创建并启动激光雷达里程计处理线程
    std::thread measurement_process(&zjloc::lidarodom::run, lio);

    // 开始ROS事件循环，等待消息回调
    ros::spin();

    // 程序结束时的清理工作
    zjloc::common::Timer::PrintAll();                                    // 打印所有计时器统计信息
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt")); // 将计时信息保存到文件

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);  // 等待3秒钟让其他线程完成
    return 0;  // 正常退出
}