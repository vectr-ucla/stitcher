#pragma once

#ifndef ROS_GOAL_CLICK_PLANNING_H
#define ROS_GOAL_CLICK_PLANNING_H
#include <ros/ros.h>

// Standard messages
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <cstdlib>
#include <mutex>
#include <vector>
#include <Eigen/Dense>

#include <HeuristicDP.h>
#include <LqmtSTITCHER.h>
#include <primitiveParams.h>
#include <Astar.h>
#include <Jps3d.h>
#include <GeometricPlanner.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <ros/package.h>
#include <fstream>

namespace Motion_Primitives {
namespace RosNode {
class LocalPlanner;
  }
}

class Motion_Primitives::RosNode::LocalPlanner {
public:

  /**
   * Constructor.
   * @param nh
   *   Ros Node handle to intialize the node.
   */
  LocalPlanner ( ros::NodeHandle nodehandle );

  /**
   * Destructor for the node.
   */
  ~LocalPlanner ( );

  static void Abort() { abort = true; }

  void Start ( );

  void Stop ( );

private: 
  void getParams();
  void pubStartAndGoal(Eigen::Vector3d target, int id);
  void pubData();
  template <typename PlannerType>
  bool pubTrajectory(PlannerType &planner, 
                     ros::Publisher &publisher,
                     const std::string &planner_name,
                     bool verbose = true);
  void pubRefPath(Eigen::Vector3d start, Eigen::Vector3d goal);
  void pubAllDPTrajOptions();
  template <typename PlannerType>
  bool pubColoredTrajectory(PlannerType &planner, 
                            ros::Publisher &publisher,
                            const std::string &planner_name);
  std_msgs::ColorRGBA getColorForVelocity(double velocity, double v_min, double v_max);
  template <typename PrimitiveType>
  void getStates(const std::vector<PrimitiveType> &primitives,
                 std::vector<Eigen::Vector3d> &positions,
                 std::vector<double> &vel_mags);
  template <typename PrimitiveType>
  bool pubPrimitive(const PrimitiveType &primitive,
                    visualization_msgs::MarkerArray &marker_array,
                    int &counter); 
  void pubWaypoints(std::vector<Eigen::Vector3d> waypoints);
  template <typename PlannerType>
  void pubBubbles(PlannerType &planner);
  template<typename PrimitiveType>
  double calcLQMTJ(const std::vector<PrimitiveType> &primitives, std::string planner_name);
  template<typename PrimitiveType>
  void saveStates(const std::vector<PrimitiveType> &primitives,
                  const std::string &file_name,
                  bool reverse = true);
  void targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent &e);

  // data members
  ros::NodeHandle nh;
  ros::Publisher pub_cloud;
  ros::Publisher pub_state;
  ros::Publisher pub_waypoints;
  ros::Publisher pub_bubbles;
  ros::Publisher pub_dp_prim_options;
  ros::Publisher pub_heuristic_dp_traj;
  ros::Publisher pub_stitcher_traj;
  ros::Publisher pub_colored_stitcher_traj;
  ros::Publisher pub_ref_path;
  ros::Subscriber target_sub;
  ros::Timer abort_timer;

  visualization_msgs::MarkerArray marker_array;

  std::unique_ptr<GeometricPlanner> geometric_planner;
  ReferencePath reference_path;
  
  static std::atomic_bool abort;

  std::mutex mtx;

  std::string name;
  std::string frame;

  bool got_reference_path;

  Motion_Primitives::primitiveParams params;

  Eigen::Vector3d pose;
  Eigen::Vector3d global_goal;

  std::vector<Eigen::Vector3d> waypoints;

  Motion_Primitives::HeuristicDP heuristic_dp;
  Motion_Primitives::LqmtSTITCHER stitcher;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<Eigen::Vector3d> start_goal_points;
};

#endif