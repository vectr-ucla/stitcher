
#include "CollisionChecker.h"
#include <iostream>

Motion_Primitives::CollisionChecker::CollisionChecker() {
  this->set_cloud = false;
  this->set_occupancy_cloud = false;
}

Motion_Primitives::CollisionChecker::~CollisionChecker() {
  // TODO: add destructor
}

void Motion_Primitives::CollisionChecker::updateTree(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  int size = cloud->points.size();
  int count = 0;

  for (int i = 0; i < size; i++) {
    // Ensure there are enough good points to perform collision detection
    if (cloud->points[i].x == cloud->points[i].x &&
        !isinf(cloud->points[i].x)) {
      count++;
      if (count > 10) {
        this->set_cloud = true;
        break;
      }
    }
  }

  if (count < 10)
    this->set_cloud = false;

  if (this->set_cloud) {
    this->mtx.lock();
    this->kdtree.setInputCloud(cloud);
    this->mtx.unlock();
  }
}

void Motion_Primitives::CollisionChecker::updateOccupancyTree(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  int size = cloud->points.size();
  int count = 0;

  for (int i = 0; i < size; i++) {
    // Ensure there are enough good points to perform collision detection
    if (cloud->points[i].x == cloud->points[i].x &&
        !isinf(cloud->points[i].x)) {
      count++;
      if (count > 1) {
        this->set_occupancy_cloud = true;
        break;
      }
    }
  }

  if (count < 1)
    this->set_occupancy_cloud = false;

  if (this->set_occupancy_cloud) {
    this->mtx.lock();
    this->occupancy_kdtree.setInputCloud(cloud);
    this->mtx.unlock();
  }
}

double Motion_Primitives::CollisionChecker::collisionCheck(
    const Eigen::Vector3d pos,
    const double buffer,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) {
  pcl::PointXYZ searchPoint;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  double collision_count = 0.;

  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  // get nearest distance to point cloud
  kdtree.radiusSearch(
      searchPoint, buffer, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1);
  collision_count += pointIdxRadiusSearch.size();
  
  return collision_count;
}

double Motion_Primitives::CollisionChecker::collisionLineCheck(Eigen::Vector3d start, 
                                                               Eigen::Vector3d goal, 
                                                               const double buffer, 
                                                               const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree){
  Eigen::Vector3d segment_vector = goal - start;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  double collision_count = 0.;

  for (double t=0; t <= 1; t+=0.01){
    Eigen::Vector3d pos = start + t*segment_vector;
    
    //count number of points within buffer
    pcl::PointXYZ searchPoint;
    searchPoint.x = pos(0);
    searchPoint.y = pos(1);
    searchPoint.z = pos(2);

    kdtree.radiusSearch(searchPoint, buffer, pointIdxRadiusSearch, pointRadiusSquaredDistance, 1);
    collision_count += pointIdxRadiusSearch.size();
    if (collision_count > 0) {
      return collision_count;
    }
  }

  return collision_count; 
}

double Motion_Primitives::CollisionChecker::distanceComparison(const Eigen::Vector3d current,
                                                               const Eigen::Vector3d primitive_pos,
                                                               const double dist_to_obs) {
  double distance_traveled;
  distance_traveled = (primitive_pos-current).norm();
  return distance_traveled - dist_to_obs;
}

double Motion_Primitives::CollisionChecker::collisionCheckMap(
    const Eigen::Vector3d pos, const double buffer) {
  this->mtx.lock();
  bool should_exit = !this->set_occupancy_cloud;
  this->mtx.unlock();

  if (should_exit)
    return -1.;

  this->mtx.lock();
  double collision_count = collisionCheck(pos, buffer, this->occupancy_kdtree);
  this->mtx.unlock();

  return collision_count;
}
