#include "CollisionChecker.h"
#include <iostream>

template <typename PrimitiveType>
bool Motion_Primitives::CollisionChecker::advanceCollisionStep(
    const PrimitiveType& primitive,
    const double buffer,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    const double v_max,
    const double tol,
    double& t_star,
    std::vector<Eigen::Vector4d>* safe_bubble_data) {
  pcl::PointXYZ searchPoint;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  Eigen::Vector3d pos = primitive.getPos(t_star);
  searchPoint.x = pos(0);
  searchPoint.y = pos(1);
  searchPoint.z = pos(2);

  kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
  double dist_to_obs = sqrt(pointRadiusSquaredDistance[0]);

  if (dist_to_obs < buffer) {
    return true;
  }

  if (safe_bubble_data) {
    Eigen::Vector4d bubble_data(pos(0), pos(1), pos(2), dist_to_obs);
    safe_bubble_data->push_back(bubble_data);
  }

  if (v_max > 0) {
    double dt = dist_to_obs / v_max;
    t_star += dt;
    return false;
  }

  int N = 1;
  int N_max = 20;
  double t = t_star;
  double t_ub = primitive.horizon;
  double t_lb = t_star;
  while (N < N_max) {
    t = 0.5 * (t_lb + t_ub);
    if (distanceComparison(pos, primitive.getPos(t), dist_to_obs) == 0 || (t_ub - t_lb) < tol) {
      break;
    }
    if (distanceComparison(pos, primitive.getPos(t), dist_to_obs) > 0) {
      t_ub = t;
    } else {
      t_lb = t;
    }
    N += 1;
  }
  t_star = t;
  return false;
}

template <typename PrimitiveType>
double Motion_Primitives::CollisionChecker::collisionCheck(
    const PrimitiveType& primitive,
    const double buffer,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    const double &v_max) {
  double t_star = 0.0;
  double collision_count = 0.;
  double tol = 1e-2;

  while (primitive.horizon-t_star > tol) {
    if (advanceCollisionStep(primitive, buffer, kdtree, v_max, tol, t_star, nullptr)) {
      collision_count += 1;
      return collision_count;
    }
  }

  return collision_count;
}

template <typename PrimitiveType>
double Motion_Primitives::CollisionChecker::collisionCheck(
    const PrimitiveType& primitive,
    const double buffer,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    std::vector<Eigen::Vector4d> & safe_bubble_data,
    const double &v_max) {
  //stores known safe bubbles along primitive path

  double t_star = 0.0;
  double collision_count = 0.;
  double tol = 1e-2;

  while (primitive.horizon-t_star > tol) {
    if (advanceCollisionStep(primitive, buffer, kdtree, v_max, tol, t_star, &safe_bubble_data)) {
      collision_count += 1;
      return collision_count;
    }
  }

  return collision_count;
}

template <typename PrimitiveType>
double Motion_Primitives::CollisionChecker::collisionCheck(
    const PrimitiveType& primitive, const double buffer, const double v_max) {
  this->mtx.lock();
  bool should_exit = !this->set_cloud;
  this->mtx.unlock();

  if (should_exit)
    return -1.;

  this->mtx.lock();
  double collision_count = collisionCheck(primitive, buffer, this->kdtree, v_max);
  this->mtx.unlock();

  return collision_count;
}

template <typename PrimitiveType>
double Motion_Primitives::CollisionChecker::collisionCheckMap(
    const PrimitiveType& primitive, const double buffer, const double v_max) {
  this->mtx.lock();
  bool should_exit = !this->set_occupancy_cloud;
  this->mtx.unlock();

  if (should_exit)
    return -1.;

  this->mtx.lock();
  double collision_count = collisionCheck(primitive, buffer, this->occupancy_kdtree, v_max);
  this->mtx.unlock();

  return collision_count;
}

template <typename PrimitiveType>
double Motion_Primitives::CollisionChecker::collisionCheckMap(
    const PrimitiveType& primitive, const double buffer, std::vector<Eigen::Vector4d> & safe_bubble_data, const double v_max) {
  this->mtx.lock();
  bool should_exit = !this->set_occupancy_cloud;
  this->mtx.unlock();

  if (should_exit)
    return -1.;

  this->mtx.lock();
  double collision_count = collisionCheck(primitive, buffer, this->occupancy_kdtree, safe_bubble_data, v_max);
  this->mtx.unlock();

  return collision_count;
}
