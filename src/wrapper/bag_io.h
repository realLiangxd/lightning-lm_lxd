//
// Created by xiang on 23-12-14.
//

#ifndef LIGHTNING_BAG_IO_H
#define LIGHTNING_BAG_IO_H

#include <functional>
#include <map>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"
#include "core/lightning_math.hpp"
#include "io/dataset_type.h"
#include "wrapper/ros_utils.h"

namespace lightning {

/**
 * ROSBAG IO
 * 指定一个包名，添加一些回调函数，就可以顺序遍历这个包
 * 现在可以指定ROS2
 */
class RosbagIO {
   public:
    explicit RosbagIO(std::string bag_file, DatasetType dataset_type = DatasetType::NCLT)
        : bag_file_(std::move(bag_file)) {
        /// handle ctrl-c
        signal(SIGINT, lightning::debug::SigHandle);    // 处理 ctrl-c 信号 
    }   // explicit 是必要的，否则会有隐式转换的问题
        // std::move() 的使用方法：将一个对象的所有权转移给另一个对象，避免不必要的拷贝

    using MsgType = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;     // 定义消息类型，rosbag2_storage::SerializedBagMessage 是ROS2中序列化消息的存储类型
    using MessageProcessFunction = std::function<bool(const MsgType &m)>;   // 定义消息处理函数类型，接受一个消息指针，返回bool值

    /// 一些方便直接使用的topics, messages
    using Scan2DHandle = std::function<bool(sensor_msgs::msg::LaserScan::SharedPtr)>;

    using PointCloud2Handle = std::function<bool(sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using LivoxCloud2Handle = std::function<bool(livox_ros_driver2::msg::CustomMsg::SharedPtr)>;
    using FullPointCloudHandle = std::function<bool(FullCloudPtr)>;
    using ImuHandle = std::function<bool(IMUPtr)>;
    using OdomHandle = std::function<bool(const OdomPtr &)>;

    /**
     * 遍历文件内容，调用回调函数
     * @param sleep_usec 每调用一个回调后的等待时间
     */
    void Go(int sleep_usec = 0);

    /// 通用处理函数
    RosbagIO &AddHandle(const std::string &topic_name, MessageProcessFunction func) {
        process_func_.emplace(topic_name, func);
        return *this;
    }

    /// point cloud 2 处理
    RosbagIO &AddPointCloud2Handle(const std::string &topic_name, PointCloud2Handle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {  // m 是序列化的消息, 需要反序列化;ROS bag 文件（.db3）本身就是一个数据库，它存储的就是序列化后的 ROS 消息。
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::SerializedMessage data(*m->serialized_data);
            seri_cloud2_.deserialize_message(&data, msg.get());

            return f(msg);
        });
    }

    /// livox 处理
    RosbagIO &AddLivoxCloudHandle(const std::string &topic_name, LivoxCloud2Handle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
            rclcpp::SerializedMessage data(*m->serialized_data);
            seri_livox_.deserialize_message(&data, msg.get());

            return f(msg);
        });
    }

    RosbagIO &AddImuHandle(const std::string &topic_name, ImuHandle f) {
        return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            rclcpp::SerializedMessage data(*m->serialized_data);
            seri_imu_.deserialize_message(&data, msg.get());

            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            return f(imu);
        });
    }

    /// odom 处理
    // RosbagIO &AddOdomHandle(const std::string &topic_name, OdomHandle f) {
    //     return AddHandle(topic_name, [f, this](const MsgType &m) -> bool {
    //         auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    //         rclcpp::SerializedMessage data(*m->serialized_data);
    //         seri_odom_.deserialize_message(&data, msg.get());

    //         /// nav_msg 的 odometry 转 odom
    //         return f(msg);
    //     });
    // }

    /// 清除现有的处理函数
    void CleanProcessFunc() { process_func_.clear(); }

   private:
    std::map<std::string, MessageProcessFunction> process_func_;

    /// 序列化
    rclcpp::Serialization<nav_msgs::msg::Odometry> seri_odom_;  //类似于ROS1中的message_filters::Subscriber
    rclcpp::Serialization<sensor_msgs::msg::Imu> seri_imu_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> seri_cloud2_;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> seri_livox_;

    std::string bag_file_;
    DatasetType dataset_type_ = DatasetType::NCLT;
};
}  // namespace lightning

#endif  // SLAM_ROS_BAG_IO_H
