#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <mutex>
#include <numeric>
#include <Eigen/Dense>


namespace Motion_Primitives {
class CollisionChecker;
}

class Motion_Primitives::CollisionChecker {
public:

	/**
	* Constructor.
	*/
	CollisionChecker (  );

	/**
	* Destructor for the node.
	*/
	~CollisionChecker ( );

  void updateTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void updateOccupancyTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  template <typename PrimitiveType>
  double collisionCheck(const PrimitiveType& primitive,
                        const double buffer,
                        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                        const double &v_max);
  template <typename PrimitiveType>
  double collisionCheck(const PrimitiveType& primitive,
                        const double buffer,
                        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
                        std::vector<Eigen::Vector4d> & safe_bubble_data,
                        const double &v_max);

  double collisionCheck(const Eigen::Vector3d pos,
                        const double buffer,
                        const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);

  template <typename PrimitiveType>
  double collisionCheck(const PrimitiveType& primitive,
                        const double buffer,
                        const double v_max = -1.0);

  template <typename PrimitiveType>
  double collisionCheckMap(const PrimitiveType& primitive,
                           const double buffer,
                           const double v_max = -1.0);
  template <typename PrimitiveType>
  double collisionCheckMap(const PrimitiveType& primitive, 
                           const double buffer, 
                           std::vector<Eigen::Vector4d> & safe_bubble_data,
                           const double v_max = -1.0);
  double collisionCheckMap(const Eigen::Vector3d pos, 
                           const double buffer);
  double collisionLineCheck(const Eigen::Vector3d start,
                            const Eigen::Vector3d end,
                            const double buffer,
                            const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);

  pcl::KdTreeFLANN<pcl::PointXYZ> occupancy_kdtree;

private: 
  double distanceComparison(const Eigen::Vector3d current,
                            const Eigen::Vector3d primitive_pos,
                            const double dist_to_obs);
  
	// data members
	std::mutex mtx;
  std::atomic<bool> set_cloud;
  std::atomic<bool> set_occupancy_cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
};

#include "CollisionChecker.tpp" //template implementation

#endif