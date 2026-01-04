#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// 注意：本版本不使用重力/IMU，仅在 base_link 下处理；地面使用“扇区-径向分桶最低z曲线+距离自适应阈值”剔除。
// 草捆检测使用 OBB 尺寸过滤（L/W/H），参数简化，便于现场调节。

class BaleEstimatorNode : public rclcpp::Node {
public:
  BaleEstimatorNode() : Node("bale_pose_estimator") {
    // 输入/坐标系/TF
    // input_topic: 点云话题
    // target_frame: 算法处理坐标系（默认 base_link），节点内部将 rslidar 点云变到此坐标系
    // tf_timeout_ms: TF 查询超时时间(ms)
    this->declare_parameter<std::string>("input_topic", "/rslidar_points");
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<int>("tf_timeout_ms", 50);

    // ROI 与体素降采样
    // r_min/r_max: 水平距离ROI(m)
    // z_min/z_max: 高度ROI(m)，以 target_frame 的 z 为准
    // voxel_leaf: 体素边长(m)
    this->declare_parameter<double>("r_min", 0.8);
    this->declare_parameter<double>("r_max", 30.0);
    this->declare_parameter<double>("z_min", -1.0);
    this->declare_parameter<double>("z_max", 3.0);
    this->declare_parameter<double>("voxel_leaf", 0.10);

    // 地面剔除（自适应阈值，剔除更多地面/矮小杂草）
    // ground_th_base: 近处阈值，建议 0.06~0.10 m
    // ground_th_slope: 随距离线性增加系数，建议 0.003~0.006 m/m
    // sector_count: 扇区数（360/sector_count 度每扇区），建议 60~90
    // bin_dr: 径向bin大小(m)，建议 0.4~0.6 m
    this->declare_parameter<double>("ground_th_base", 0.08);
    this->declare_parameter<double>("ground_th_slope", 0.004);
    this->declare_parameter<int>("sector_count", 72);       // 5 deg/sector
    this->declare_parameter<double>("bin_dr", 0.5);

    // 聚类（容差随距离线性增大）
    // cluster_tol_base: 基准容差(m)，建议 0.28~0.35
    // cluster_tol_slope: 随距离增加系数(m/m)，建议 0.008~0.012
    // cluster_split: 近/远分界(m)，用于两段聚类，建议 12 m
    // min_cluster_size: 最小聚类点数，建议 30~60（远处可降）
    this->declare_parameter<double>("cluster_tol_base", 0.30);
    this->declare_parameter<double>("cluster_tol_slope", 0.010);
    this->declare_parameter<double>("cluster_split", 12.0);
    this->declare_parameter<int>("min_cluster_size", 30);

    // 尺寸过滤（OBB）：L（最大边）≈ 草捆长度；D≈(W+H)/2≈ 草捆直径
    // 60cm 直径、1.0m 长度建议：
    // size_diam_min/max: 直径范围(m) 0.54~0.66
    // size_len_min/max: 长度范围(m) 0.90~1.10
    this->declare_parameter<double>("size_diam_min", 0.54);
    this->declare_parameter<double>("size_diam_max", 0.66);
    this->declare_parameter<double>("size_len_min", 0.90);
    this->declare_parameter<double>("size_len_max", 1.10);

    // 可视化
    this->declare_parameter<bool>("publish_text_marker", true);

    // 读取参数
    input_topic_         = this->get_parameter("input_topic").as_string();
    target_frame_        = this->get_parameter("target_frame").as_string();
    tf_timeout_ms_       = this->get_parameter("tf_timeout_ms").as_int();

    r_min_               = this->get_parameter("r_min").as_double();
    r_max_               = this->get_parameter("r_max").as_double();
    z_min_               = this->get_parameter("z_min").as_double();
    z_max_               = this->get_parameter("z_max").as_double();
    voxel_leaf_          = this->get_parameter("voxel_leaf").as_double();

    ground_th_base_      = this->get_parameter("ground_th_base").as_double();
    ground_th_slope_     = this->get_parameter("ground_th_slope").as_double();
    sector_count_        = this->get_parameter("sector_count").as_int();
    bin_dr_              = this->get_parameter("bin_dr").as_double();

    cluster_tol_base_    = this->get_parameter("cluster_tol_base").as_double();
    cluster_tol_slope_   = this->get_parameter("cluster_tol_slope").as_double();
    cluster_split_       = this->get_parameter("cluster_split").as_double();
    min_cluster_size_    = this->get_parameter("min_cluster_size").as_int();

    size_diam_min_       = this->get_parameter("size_diam_min").as_double();
    size_diam_max_       = this->get_parameter("size_diam_max").as_double();
    size_len_min_        = this->get_parameter("size_len_min").as_double();
    size_len_max_        = this->get_parameter("size_len_max").as_double();

    publish_text_marker_ = this->get_parameter("publish_text_marker").as_bool();

    // TF2
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // IO
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BaleEstimatorNode::cloudCallback, this, std::placeholders::_1));

    pose_pub_   = this->create_publisher<geometry_msgs::msg::PoseArray>("/bale_poses", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/bale_markers", 10);

    RCLCPP_INFO(this->get_logger(),
      "Bale estimator (base_link). Sub: %s, ROI r=[%.2f,%.2f] z=[%.2f,%.2f]",
      input_topic_.c_str(), r_min_, r_max_, z_min_, z_max_);
  }

private:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;

  // 简易 OBB 计算（PCA 轴 + 轴向 min/max）
  static bool computeOBB(const CloudT::Ptr& c,
                         Eigen::Vector3f& center,
                         Eigen::Matrix3f& axes,
                         Eigen::Vector3f& extents) {
    if (!c || c->empty()) return false;

    Eigen::Vector3f centroid(0,0,0);
    for (const auto& p : c->points) centroid += Eigen::Vector3f(p.x,p.y,p.z);
    centroid /= static_cast<float>(c->size());

    pcl::PCA<PointT> pca;
    pca.setInputCloud(c);
    axes = pca.getEigenVectors(); // 列向量为主轴
    if (!axes.allFinite()) return false;

    float min_u[3] = {+1e9f,+1e9f,+1e9f};
    float max_u[3] = {-1e9f,-1e9f,-1e9f};
    for (const auto& p : c->points) {
      Eigen::Vector3f d = Eigen::Vector3f(p.x,p.y,p.z) - centroid;
      Eigen::Vector3f u = axes.transpose() * d;
      for (int k = 0; k < 3; ++k) {
        min_u[k] = std::min(min_u[k], u[k]);
        max_u[k] = std::max(max_u[k], u[k]);
      }
    }
    extents = Eigen::Vector3f(max_u[0]-min_u[0], max_u[1]-min_u[1], max_u[2]-min_u[2]);
    Eigen::Vector3f center_local((max_u[0]+min_u[0])*0.5f,
                                 (max_u[1]+min_u[1])*0.5f,
                                 (max_u[2]+min_u[2])*0.5f);
    center = centroid + axes * center_local;

    if (axes.determinant() < 0) { axes.col(2) = axes.col(0).cross(axes.col(1)); }
    return extents.allFinite() && center.allFinite() && axes.allFinite();
  }

  // 地面剔除：扇区×径向分桶最低z + 距离自适应阈值
  CloudT::Ptr removeGroundSectorBin(const CloudT::Ptr& cloud_ds) {
    const int   SECT = std::max(12, sector_count_);
    const float DR   = static_cast<float>(std::max(0.1, bin_dr_));
    const float RMAX = static_cast<float>(r_max_);
    const int   NBIN = std::max(1, int(std::ceil(RMAX / DR)));

    std::vector<std::vector<float>> zmin(SECT, std::vector<float>(NBIN, std::numeric_limits<float>::infinity()));

    // pass 1: 统计每个扇区-半径bin 的最小z
    for (const auto& p : cloud_ds->points) {
      const float r = std::hypot(p.x, p.y);
      if (r < 0.f || r > RMAX) continue;
      const int ib = std::min(NBIN - 1, std::max(0, int(r / DR)));
      float ang = std::atan2(p.y, p.x); // [-pi, pi]
      if (ang < 0) ang += 2.f * float(M_PI);
      const int is = std::min(SECT - 1, std::max(0, int(ang / (2.f * float(M_PI)) * SECT)));
      zmin[is][ib] = std::min(zmin[is][ib], p.z);
    }

    // pass 2: 剔除近地面点（z - zmin > th(r) 保留）
    CloudT::Ptr out(new CloudT);
    out->reserve(cloud_ds->size());
    for (const auto& p : cloud_ds->points) {
      const float r = std::hypot(p.x, p.y);
      if (r < 0.f || r > RMAX) continue;
      const int ib = std::min(NBIN - 1, std::max(0, int(r / DR)));
      float ang = std::atan2(p.y, p.x); 
      if (ang < 0) ang += 2.f * float(M_PI);
      const int is = std::min(SECT - 1, std::max(0, int(ang / (2.f * float(M_PI)) * SECT)));
      const float z0 = zmin[is][ib];
      if (!std::isfinite(z0)) { out->push_back(p); continue; }
      const float th = static_cast<float>(ground_th_base_ + ground_th_slope_ * double(r));
      if ((p.z - z0) > th) out->push_back(p);
    }
    return out;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto clk = this->get_clock(); // for THROTTLE

    // 1) TF 到 target_frame（默认 base_link）
    sensor_msgs::msg::PointCloud2 msg_tf;
    if (!target_frame_.empty() && target_frame_ != msg->header.frame_id) {
      try {
        auto tf = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_nanoseconds(static_cast<uint64_t>(tf_timeout_ms_) * 1000ull * 1000ull));
        tf2::doTransform(*msg, msg_tf, tf);
        msg_tf.header.frame_id = target_frame_;
        RCLCPP_INFO_THROTTLE(this->get_logger(), *clk, 2000,
          "TF applied: %s -> %s", msg->header.frame_id.c_str(), target_frame_.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clk, 2000,
          "TF %s->%s failed: %s", msg->header.frame_id.c_str(), target_frame_.c_str(), e.what());
        return;
      }
    } else {
      msg_tf = *msg;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clk, 2000,
        "No TF, processing in frame '%s'", msg_tf.header.frame_id.c_str());
    }

    // 2) 转PCL + ROI 裁剪
    CloudT::Ptr cloud(new CloudT);
    pcl::fromROSMsg(msg_tf, *cloud);
    if (cloud->empty()) { publishEmpty(msg_tf.header.frame_id, rclcpp::Time(msg_tf.header.stamp)); return; }

    CloudT::Ptr roi(new CloudT);
    roi->reserve(cloud->size());
    const float RMIN = static_cast<float>(r_min_), RMAX = static_cast<float>(r_max_);
    const float ZMIN = static_cast<float>(z_min_), ZMAX = static_cast<float>(z_max_);
    for (const auto& p : cloud->points) {
      const float r = std::hypot(p.x, p.y);
      if (r >= RMIN && r <= RMAX && p.z >= ZMIN && p.z <= ZMAX) {
        roi->push_back(p);
      }
    }
    if (roi->size() < 30) { publishEmpty(msg_tf.header.frame_id, rclcpp::Time(msg_tf.header.stamp)); return; }

    // 3) 体素降采样
    CloudT::Ptr ds(new CloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(roi);
    vg.setLeafSize(static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_), static_cast<float>(voxel_leaf_));
    vg.filter(*ds);
    if (ds->size() < 30) { publishEmpty(msg_tf.header.frame_id, rclcpp::Time(msg_tf.header.stamp)); return; }

    // 4) 地面剔除（扇区-径向最低z + 自适应阈值）
    CloudT::Ptr nonground = removeGroundSectorBin(ds);
    if (nonground->size() < 30) { publishEmpty(msg_tf.header.frame_id, rclcpp::Time(msg_tf.header.stamp)); return; }

    // 5) 近/远两段聚类（容差线性随距离增加）
    CloudT::Ptr near_cloud(new CloudT), far_cloud(new CloudT);
    const float SPLIT = static_cast<float>(cluster_split_);
    for (const auto& p : nonground->points) {
      (std::hypot(p.x, p.y) <= SPLIT ? near_cloud : far_cloud)->push_back(p);
    }

    auto computeTol = [&](float r_mid) -> double {
      return cluster_tol_base_ + cluster_tol_slope_ * static_cast<double>(r_mid);
    };
    const float r_near_mid = std::max(RMIN, std::min(SPLIT, (RMIN + SPLIT) * 0.5f));
    const float r_far_mid  = std::max(SPLIT, std::min(RMAX, (SPLIT + RMAX) * 0.5f));
    const double tol_near  = computeTol(r_near_mid);
    const double tol_far   = computeTol(r_far_mid);

    std::vector<CloudT::Ptr> clusters;
    clusterOne(near_cloud, tol_near, clusters);
    clusterOne(far_cloud,  tol_far,  clusters);

    // 6) OBB 尺寸过滤 + 输出
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = msg_tf.header.frame_id;
    pose_array.header.stamp    = rclcpp::Time(msg_tf.header.stamp);

    visualization_msgs::msg::MarkerArray markers;
    { visualization_msgs::msg::Marker del; del.action = visualization_msgs::msg::Marker::DELETEALL; markers.markers.push_back(del); }

    const double DMIN = size_diam_min_, DMAX = size_diam_max_;
    const double LMIN = size_len_min_,  LMAX = size_len_max_;
    int id = 0;

    for (const auto& c : clusters) {
      if (!c || c->size() < static_cast<size_t>(min_cluster_size_)) continue;

      Eigen::Vector3f center; Eigen::Matrix3f axes; Eigen::Vector3f extents;
      if (!computeOBB(c, center, axes, extents)) continue;

      // 排序 extents: L >= W >= H
      int idx[3] = {0,1,2};
      auto cmp = [&](int a, int b){ return extents[a] > extents[b]; };
      std::sort(idx, idx+3, cmp);
      Eigen::Vector3f sorted_extents(extents[idx[0]], extents[idx[1]], extents[idx[2]]);
      Eigen::Matrix3f sorted_axes;
      sorted_axes.col(0) = axes.col(idx[0]);
      sorted_axes.col(1) = axes.col(idx[1]);
      sorted_axes.col(2) = axes.col(idx[2]);

      const double L = sorted_extents(0);
      const double W = sorted_extents(1);
      const double H = sorted_extents(2);
      const double D = 0.5 * (W + H); // 直径估计

      if (!(L >= LMIN && L <= LMAX && D >= DMIN && D <= DMAX)) continue;

      Eigen::Quaternionf q(sorted_axes); q.normalize();

      geometry_msgs::msg::Pose pose;
      pose.position.x = center.x();
      pose.position.y = center.y();
      pose.position.z = center.z();
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      pose_array.poses.push_back(pose);

      visualization_msgs::msg::Marker m;
      m.header = pose_array.header;
      m.ns = "bale_bbox";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = static_cast<float>(L);
      m.scale.y = static_cast<float>(W);
      m.scale.z = static_cast<float>(H);
      m.color.r = 0.2f; m.color.g = 0.7f; m.color.b = 1.0f; m.color.a = 0.65f;
      m.lifetime = rclcpp::Duration(0, 0);
      markers.markers.push_back(m);

      if (publish_text_marker_) {
        std::ostringstream ss; ss.setf(std::ios::fixed); ss.precision(2);
        ss << "L=" << L << "m\nD~" << D << "m";
        visualization_msgs::msg::Marker t;
        t.header = pose_array.header;
        t.ns = "bale_text";
        t.id = 10000 + id;
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.pose = pose; t.pose.position.z += static_cast<float>(H * 0.6);
        t.scale.z = 0.20f;
        t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 0.95f;
        t.text = ss.str();
        t.lifetime = rclcpp::Duration(0, 0);
        markers.markers.push_back(t);
      }
      ++id;
    }

    pose_pub_->publish(pose_array);
    marker_pub_->publish(markers);
  }

  void clusterOne(const CloudT::Ptr& cloud, double tol, std::vector<CloudT::Ptr>& out) {
    if (!cloud || cloud->empty()) return;

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(static_cast<float>(tol));
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& idx : cluster_indices) {
      CloudT::Ptr c(new CloudT);
      c->points.reserve(idx.indices.size());
      for (int i : idx.indices) c->points.push_back(cloud->points[i]);
      out.push_back(c);
    }
  }

  void publishEmpty(const std::string& frame_id, const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseArray pa;
    pa.header.frame_id = frame_id;
    pa.header.stamp = stamp;
    pose_pub_->publish(pa);

    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(del);
    marker_pub_->publish(ma);
  }

  // 参数
  std::string input_topic_;
  std::string target_frame_;
  int    tf_timeout_ms_{50};

  double r_min_, r_max_;
  double z_min_, z_max_;
  double voxel_leaf_;

  double ground_th_base_;
  double ground_th_slope_;
  int    sector_count_;
  double bin_dr_;

  double cluster_tol_base_;
  double cluster_tol_slope_;
  double cluster_split_;
  int    min_cluster_size_;

  double size_diam_min_, size_diam_max_;
  double size_len_min_,  size_len_max_;
  bool   publish_text_marker_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr     pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BaleEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
