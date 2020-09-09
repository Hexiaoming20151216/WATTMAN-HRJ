
#ifndef POINT2PLANEICP_H
#define POINT2PLANEICP_H


#include <iostream>
#include <chrono>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

class Point2PlaneICP {
 public:
  Point2PlaneICP();
  ~Point2PlaneICP(){}

  bool setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_source);
  bool setInputTarget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_target);
  bool align(pcl::PointCloud<pcl::PointXYZRGB>& pointCloud_transed);
  Eigen::Matrix4f  getFinalTransformation();
//  Eigen::Affine3d                 final_transform_;
  Eigen::Matrix4f                final_transform_;
 private:
  void addNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_with_normals);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_;

};

#endif //POINT2PLANEICP_H
