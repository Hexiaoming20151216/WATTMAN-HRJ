#include <iostream>
#include <chrono>
#include <pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "point2plane_ICP.h"


Eigen::Matrix4d get_transform(Eigen::Quaterniond quat_imu_a, Eigen::Vector3d pos_a,
                              Eigen::Quaterniond quat_imu_b, Eigen::Vector3d pos_b
                              )
{
    Eigen::Matrix3d R_a = Eigen::Matrix3d(quat_imu_a);
    Eigen::Matrix4d T_a = Eigen::Matrix4d::Identity();
    T_a.block<3,3>(0,0) = R_a;
    T_a.block<3,1>(0,3) = pos_a;

    Eigen::Matrix3d R_b = Eigen::Matrix3d(quat_imu_b);
    Eigen::Matrix4d T_b = Eigen::Matrix4d::Identity();
    T_b.block<3,3>(0,0) = R_b;
    T_b.block<3,1>(0,3) = pos_b;

    return T_b*T_a.inverse();
}

int main() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_002(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_003(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_registered(new pcl::PointCloud<pcl::PointXYZRGB>);


    //
    //*打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("../pcd/cloud_002.pcd", *cloud_002) == -1) {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
        return(-1);
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("../pcd/cloud_003.pcd", *cloud_003) == -1) {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
        return(-1);
    }
    std::cout << "Loaded:" << cloud_002->width*cloud_002->height<<"data points from cloud_002.pcd"<< std::endl;
    std::cout << "Loaded:" << cloud_003->width*cloud_003->height<<"data points from cloud_003.pcd"<< std::endl;
    pcl::io::savePLYFile("../ply_data/cloud_002.ply", *cloud_002);
    pcl::io::savePLYFile("../ply_data/cloud_003.ply", *cloud_003);

   ///ICP
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    Point2PlaneICP icp;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_002_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_002);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);//体素大小设置为20*20*20cm
    sor.filter(*cloud_002_filtered_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_003_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_003);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);//体素大小设置为20*20*20cm
    sor.filter(*cloud_003_filtered_cloud);

// Set the input source and target
    icp.setInputCloud (cloud_002_filtered_cloud);
    icp.setInputTarget (cloud_003_filtered_cloud);

    icp.align (*cloud_registered);
// Obtain the transformation that aligned cloud_source to cloud_source_registered
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    std::cout<< "ICP  cost time: " <<  transformation << std::endl;


    std::chrono::high_resolution_clock::time_point post = std::chrono::high_resolution_clock::now();
    std::cout<< "ICP  cost time: " << std::chrono::duration_cast<std::chrono::duration<double>>(post - now).count() * 1000 << " ms" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_002_trans(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*cloud_002, *cloud_002_trans, transformation);
    pcl::io::savePLYFile("../ply_data/cloud_002_trans.ply", *cloud_002_trans);
    pcl::io::savePLYFile("../ply_data/cloud_registered.ply", *cloud_registered);



    return 0;
}
