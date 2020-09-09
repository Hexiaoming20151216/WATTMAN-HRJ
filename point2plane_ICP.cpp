#include "point2plane_ICP.h"

Point2PlaneICP::Point2PlaneICP()
{
  cloud_source_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cloud_source_->clear();
  cloud_target_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cloud_target_->clear();
  final_transform_ = Eigen::Matrix4f::Identity();
}

bool Point2PlaneICP::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_source)
{
  *cloud_source_ = *cloud_source;
  return true;
}
bool Point2PlaneICP::setInputTarget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_target)
{
  *cloud_target_ = *cloud_target;
  return true;
}

void Point2PlaneICP::addNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud_with_normals)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  searchTree->setInputCloud ( cloud );

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch ( 15 );
  normalEstimator.compute ( *normals );

  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}

Eigen::Matrix4f  Point2PlaneICP::getFinalTransformation()
{
  return final_transform_;
}
bool Point2PlaneICP::align(pcl::PointCloud<pcl::PointXYZRGB>& pointCloud_transed)
{
  // transformed source ---> target
//  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );
//  cloud_source_trans = cloud_source_;

  // prepare could with normals
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
//  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );


  addNormal( cloud_source_, cloud_source_normals );
  addNormal( cloud_target_, cloud_target_normals );
//  addNormal( cloud_source_trans, cloud_source_trans_normals );


  pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  icp->setInputSource ( cloud_source_normals ); // not cloud_source, but cloud_source_trans!
  icp->setInputTarget ( cloud_target_normals );
  icp->setMaxCorrespondenceDistance(1.1);//设置对应点对之间的最大距离（此值对配准结果影响较大）。
  icp->setTransformationEpsilon(1e-10);//设置两次变化矩阵之间的差值（一般设置为1e-10即可）；
  icp->setEuclideanFitnessEpsilon(0.1);//设置收敛条件是icp均方误差和小于阈值， 停止迭代；
  icp->setMaximumIterations(5);//最大迭代次数，icp是一个迭代的方法，最多迭代这些次；

//  while ( !viewer->wasStopped () )
  {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    icp->align ( *cloud_source_normals ); // use cloud with normals for ICP
        std::chrono::high_resolution_clock::time_point post = std::chrono::high_resolution_clock::now();
        std::cout<< "ICP  cost time: " << std::chrono::duration_cast<std::chrono::duration<double>>(post - now).count() * 1000 << " ms" << std::endl;

    if ( icp->hasConverged() )
    {
      // use cloud without normals for visualizatoin
      pcl::transformPointCloud ( *cloud_source_, pointCloud_transed, icp->getFinalTransformation() );
      final_transform_ = icp->getFinalTransformation();
      return true;
    }
    else{
      std::cerr << "Not converged." << std::endl;
      return false;
    }
  }

}