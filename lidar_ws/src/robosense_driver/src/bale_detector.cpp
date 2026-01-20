#include <memory>
#include <cmath>
#include <limits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

class SimpleBaleLocator : public rclcpp::Node
{
public:
  SimpleBaleLocator()
  : Node("simple_bale_locator")
  {
    // 参数（可以后续改成 declare_parameter）
    min_range_ = 2.0f;
    max_range_ = 10.0f;

    // 草捆尺寸（圆捆：直径约0.6m，长约1.2m）
    // 这里给个宽松范围，后续可根据实测调整
    bale_min_long_ = 0.7f;   // 包围盒较长边下限
    bale_max_long_ = 1.6f;   // 包围盒较长边上限
    bale_min_short_ = 0.3f;  // 包围盒较短边下限
    bale_max_short_ = 0.9f;  // 包围盒较短边上限

    cluster_tolerance_ = 0.5f;   // 聚类半径
    cluster_min_size_ = 30;      // 最小点数
    cluster_max_size_ = 5000;    // 最大点数

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points",
      rclcpp::SensorDataQoS(),
      std::bind(&SimpleBaleLocator::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "SimpleBaleLocator (with clustering) subscribed to /rslidar_points");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->width * msg->height == 0) {
      return;
    }

    // 1. ROS2 PointCloud2 -> PCL 点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_in);

    if (cloud_in->empty()) {
      return;
    }

    // 2. 简单 ROI 过滤：只保留前方 2~10m 的点（在雷达坐标系 XY 平面上）
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_roi->reserve(cloud_in->size());

    for (const auto & p : cloud_in->points) {
      // 先过滤 NaN / Inf
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      // 假设x为前方
      if (p.x <= 0.0f) continue;

      float r_xy = std::sqrt(p.x * p.x + p.y * p.y);
      if (r_xy < min_range_ || r_xy > max_range_) continue;

      // 这里暂时不对z做过滤，需要的话可以再加
      cloud_roi->points.push_back(p);
    }

    if (cloud_roi->points.size() < static_cast<size_t>(cluster_min_size_)) {
      return;
    }

    // 3. 使用 PCL 的欧式聚类
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_roi);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setMaxClusterSize(cluster_max_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_roi);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
      return;
    }

    // 4. 遍历所有cluster，筛选“草捆候选”，选出最近的那个
    bool found_candidate = false;
    float best_r = std::numeric_limits<float>::max();
    float best_cx = 0.0f;
    float best_cy = 0.0f;
    float best_cz = 0.0f;

    for (const auto & indices : cluster_indices) {
      if (indices.indices.empty()) continue;

      // 4.1 计算该聚类的质心和包围盒
      float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();

      for (int idx : indices.indices) {
        const auto & p = cloud_roi->points[idx];
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;

        if (p.x < min_x) min_x = p.x;
        if (p.x > max_x) max_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.y > max_y) max_y = p.y;
        if (p.z < min_z) min_z = p.z;
        if (p.z > max_z) max_z = p.z;
      }

      const size_t N = indices.indices.size();
      float cx = sum_x / static_cast<float>(N);
      float cy = sum_y / static_cast<float>(N);
      float cz = sum_z / static_cast<float>(N);

      float dx = max_x - min_x;
      float dy = max_y - min_y;
      float dz = max_z - min_z;

      // 4.2 在XY平面上，取较长边和较短边进行草捆尺寸判断
      float d_long = std::max(dx, dy);
      float d_short = std::min(dx, dy);

      // 草捆候选的粗略尺寸过滤
      if (d_long < bale_min_long_ || d_long > bale_max_long_) {
        continue;
      }
      if (d_short < bale_min_short_ || d_short > bale_max_short_) {
        continue;
      }

      // 可选：对高度范围做个基本过滤，避免特别扁/特别高的东西
      // 这里给一个很宽的范围
      if (dz < 0.2f || dz > 1.5f) {
        continue;
      }

      // 4.3 计算该候选质心相对雷达的水平距离
      float r_xy = std::sqrt(cx * cx + cy * cy);

      // 理论上r_xy已经在[2,10]内，但仍可再判断一次（防御性）
      if (r_xy < min_range_ || r_xy > max_range_) {
        continue;
      }

      // 选择最近的草捆候选
      if (r_xy < best_r) {
        best_r = r_xy;
        best_cx = cx;
        best_cy = cy;
        best_cz = cz;
        found_candidate = true;
      }
    }

    // 5. 输出最近草捆候选的R和θ
    if (found_candidate) {
      float angle_rad = std::atan2(best_cy, best_cx);
      float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500, // 500ms 打印一次
        "Nearest bale candidate: R=%.2f m, angle=%.1f deg, centroid=(%.2f, %.2f, %.2f)",
        best_r, angle_deg, best_cx, best_cy, best_cz);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  float min_range_;
  float max_range_;

  // 草捆尺寸门限
  float bale_min_long_;
  float bale_max_long_;
  float bale_min_short_;
  float bale_max_short_;

  float cluster_tolerance_;
  int cluster_min_size_;
  int cluster_max_size_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleBaleLocator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
