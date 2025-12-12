#include "CollisionChecker.h"
#include <iostream>

template <typename PrimitiveType>
double Motion_Primitives::CollisionChecker::collisionCheck(
    const PrimitiveType& primitive,
    const double buffer,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree,
    const double &v_max) {
  pcl::PointXYZ searchPoint;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  double t_star = 0.0;
  std::vector<double> t_bound = {0., primitive.horizon};
  double collision_count = 0.;

  Eigen::Vector3d pos;
  double dist_to_obs = std::numeric_limits<double>::infinity();
  double tol = 1e-2;

  while (primitive.horizon-t_star > tol) {
    pos = primitive.getPos(t_star);

    searchPoint.x = pos(0);
    searchPoint.y = pos(1);
    searchPoint.z = pos(2);
    
    //find distance to closest obstacle
    kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    dist_to_obs = sqrt(pointRadiusSquaredDistance[0]);

    //if distance in unsafe buffer zone -> collision
    if (dist_to_obs < buffer){
      collision_count += 1;
      return collision_count;
    } 

    //if v_max is defined, increment t_star based on distance to closest obstacle
    if (v_max > 0) {
      double dt = dist_to_obs/v_max; 
      t_star += dt;
    }

    //else find next t_star using bisection search (no known max speed)
    //note: using bisection search can fail at edge cases where a primitive exits and re-enters the "closest obs bubble" as this method only finds a single root subject to t bounds
    //if there is no known max speed, consider using a different root finding method for incrementing t to ensure safety
    else{
      int N = 1;
      int N_max = 20;
      double t, t_ub, t_lb;
      t_ub = t_bound[1];
      t_lb = t_star;
      while (N < N_max){
        t = 0.5*(t_lb+t_ub);
        if (distanceComparison(pos, primitive.getPos(t), dist_to_obs) == 0 || (t_ub - t_lb) < tol){
          break;
        } 
        else if (distanceComparison(pos, primitive.getPos(t), dist_to_obs) > 0){
          t_ub = t;
        } 
        else {
          t_lb = t;
        }
        N += 1;
      }
      t_star = t;
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
  
  pcl::PointXYZ searchPoint;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  double t_star = 0.0;
  std::vector<double> t_bound = {0., primitive.horizon};
  double collision_count = 0.;

  Eigen::Vector3d pos;
  double dist_to_obs = std::numeric_limits<double>::infinity();
  double tol = 1e-2;

  while (primitive.horizon-t_star > tol) {
    pos = primitive.getPos(t_star);

    searchPoint.x = pos(0);
    searchPoint.y = pos(1);
    searchPoint.z = pos(2);
    
    //find distance to closest obstacle
    kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    dist_to_obs = sqrt(pointRadiusSquaredDistance[0]);

    //if distance in unsafe buffer zone -> collision
    if (dist_to_obs < buffer){
      collision_count += 1;
      return collision_count;
    } 

    //store safe bubble if not in collision
    Eigen::Vector4d bubble_data(pos(0), pos(1), pos(2), dist_to_obs);
    safe_bubble_data.push_back(bubble_data);

    //if v_max is defined, increment t_star based on distance to closest obstacle
    if (v_max > 0) {
      double dt = dist_to_obs/v_max; 
      t_star += dt;
    }

    //else find next t_star using bisection search (no known max speed)
    else{
    int N = 1;
    int N_max = 20;
    double t, t_ub, t_lb;
    t_ub = t_bound[1];
    t_lb = t_star;
    while (N < N_max){
      t = 0.5*(t_lb+t_ub);
      if (distanceComparison(pos, primitive.getPos(t), dist_to_obs) == 0 || (t_ub - t_lb) < tol){
        break;
      } 
      else if (distanceComparison(pos, primitive.getPos(t), dist_to_obs) > 0){
        t_ub = t;
      } 
      else {
        t_lb = t;
      }
      N += 1;
    }
    t_star = t;
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
