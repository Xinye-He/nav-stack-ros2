// robosense_node.cpp
#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

// 使用 PCL 或自定义点云
typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;
using namespace std::chrono_literals;

// 同步队列（用于双缓冲）
SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

// 全局发布器
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
rclcpp::Logger logger = rclcpp::get_logger("robosense_driver");

// 回调函数（驱动调用）
std::shared_ptr<PointCloudMsg> driverGetPointCloudCallback() {
  auto msg = free_cloud_queue.pop();
  return msg ? msg : std::make_shared<PointCloudMsg>();
}

void driverReturnPointCloudCallback(std::shared_ptr<PointCloudMsg> msg) {
  stuffed_cloud_queue.push(msg);
}

void exceptionCallback(const Error& code) {
  RCLCPP_WARN(logger, "RoboSense Driver Error: %s", code.toString().c_str());
}

// 将 RoboSense 点云转为 ROS 2 PointCloud2
sensor_msgs::msg::PointCloud2::SharedPtr convertToROS2PointCloud(const std::shared_ptr<PointCloudMsg>& rs_cloud) {
  auto cloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();

  cloud2->header.stamp = rclcpp::Time(rs_cloud->timestamp);  // 时间戳（需确保单位为纳秒）
  cloud2->header.frame_id = "rslidar";                        // 坐标系名称
  cloud2->height = 1;
  cloud2->width = rs_cloud->points.size();
  cloud2->is_dense = false;
  cloud2->is_bigendian = false;

  // 定义字段：x, y, z, intensity
  sensor_msgs::msg::PointField x_field, y_field, z_field, intensity_field;
  x_field.name = "x"; x_field.offset = 0; x_field.datatype = sensor_msgs::msg::PointField::FLOAT32; x_field.count = 1;
  y_field.name = "y"; y_field.offset = 4; y_field.datatype = sensor_msgs::msg::PointField::FLOAT32; y_field.count = 1;
  z_field.name = "z"; z_field.offset = 8; z_field.datatype = sensor_msgs::msg::PointField::FLOAT32; z_field.count = 1;
  intensity_field.name = "intensity"; intensity_field.offset = 12; 
  intensity_field.datatype = sensor_msgs::msg::PointField::FLOAT32; intensity_field.count = 1;

  // 使用 push_back 更可靠（避免 initializer list 类型推导问题）
  cloud2->fields.clear();
  cloud2->fields.push_back(x_field);
  cloud2->fields.push_back(y_field);
  cloud2->fields.push_back(z_field);
  cloud2->fields.push_back(intensity_field);

  cloud2->point_step = 16;
  cloud2->row_step = cloud2->point_step * cloud2->width;

  // 填充数据
  cloud2->data.resize(cloud2->row_step);
  for (size_t i = 0; i < rs_cloud->points.size(); ++i) {
    const auto& p = rs_cloud->points[i];
    memcpy(&cloud2->data[i * 16], &p.x, 4);
    memcpy(&cloud2->data[i * 16 + 4], &p.y, 4);
    memcpy(&cloud2->data[i * 16 + 8], &p.z, 4);
    memcpy(&cloud2->data[i * 16 + 12], &p.intensity, 4);
  }

  return cloud2;
}

class RoboSenseNode : public rclcpp::Node {
public:
  RoboSenseNode() : Node("robosense_driver") {
    logger = this->get_logger();

    // 创建发布器
    cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_points", 10);

    // 初始化驱动参数
    RSDriverParam param;
    param.input_type = InputType::ONLINE_LIDAR;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;
    param.lidar_type = LidarType::RSE1;  // ⚠️ 你的雷达型号！如 RSLIDAR_16, RSLIDAR_32, RSAIRY 等
    param.print();

    // 创建驱动
    driver_.reset(new LidarDriver<PointCloudMsg>());
    driver_->regPointCloudCallback(driverGetPointCloudCallback, driverReturnPointCloudCallback);
    driver_->regExceptionCallback(exceptionCallback);

    if (!driver_->init(param)) {
      RCLCPP_FATAL(this->get_logger(), "Driver initialization failed!");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "RoboSense driver initialized successfully.");

    // 启动驱动
    driver_->start();
    RCLCPP_INFO(this->get_logger(), "RoboSense driver started.");

    // 启动点云处理线程
    processing_thread_ = std::thread(&RoboSenseNode::processClouds, this);
  }

  ~RoboSenseNode() {
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (driver_) {
      driver_->stop();
    }
  }

private:
  void processClouds() {
    while (rclcpp::ok()) {
      auto rs_cloud = stuffed_cloud_queue.popWait();
      if (!rs_cloud) continue;

      // 转换为 ROS 2 点云
      auto ros_cloud = convertToROS2PointCloud(rs_cloud);
      cloud_pub->publish(*ros_cloud);

      RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", rs_cloud->points.size());

      // 归还空点云对象
      free_cloud_queue.push(rs_cloud);
    }
  }

  std::unique_ptr<LidarDriver<PointCloudMsg>> driver_;
  std::thread processing_thread_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // 预分配一些点云对象（提高性能）
  for (int i = 0; i < 10; ++i) {
    free_cloud_queue.push(std::make_shared<PointCloudMsg>());
  }

  auto node = std::make_shared<RoboSenseNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
