#include "ROS_Goal_Click_Planning.h"

std::atomic_bool Motion_Primitives::RosNode::LocalPlanner::abort(false);

Motion_Primitives::RosNode::LocalPlanner::LocalPlanner(ros::NodeHandle nodehandle) 
    : nh(nodehandle) {
  this->abort = false;

  getParams();

  this->frame = "odom";

  //publishing setpoint and trajectory
  this->pub_cloud = this->nh.advertise<sensor_msgs::PointCloud2>("cloud",1,true);
  this->pub_state = this->nh.advertise<visualization_msgs::MarkerArray>("start_end_state", 1);
  this->pub_waypoints = this->nh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
  this->pub_bubbles = this->nh.advertise<visualization_msgs::MarkerArray>("bubbles", 1);
  this->pub_dp_prim_options = this->nh.advertise<visualization_msgs::MarkerArray>("dynamic_program_all_primitives", 1);
  this->pub_ref_path = this->nh.advertise<nav_msgs::Path>("waypoint_path", 1);
  this->pub_heuristic_dp_traj = this->nh.advertise<nav_msgs::Path>("heuristic_dp_trajectory", 1);
  this->pub_stitcher_traj = this->nh.advertise<nav_msgs::Path>("stitcher_trajectory", 1);
  this->pub_colored_stitcher_traj = this->nh.advertise<visualization_msgs::MarkerArray>("colored_stitcher_trajectory", 1);

  this->target_sub = this->nh.subscribe(
      "move_base_simple/goal", 1,
      &Motion_Primitives::RosNode::LocalPlanner::targetCallback, this);

  this->abort_timer = this->nh.createTimer(
      ros::Duration(0.01),
      &Motion_Primitives::RosNode::LocalPlanner::timerCallback, this);

  std::string pcd_file; 
  nh.param("pcd_file", pcd_file, std::string("lab_map"));

  //Read the point cloud from pcd
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcloud) == -1)
  {
    PCL_ERROR ("Couldn't read pcd file \n");
  }

  //load point cloud to occupancy tree for collision checking and reference path generation
  this->cloud = *pcloud;
  this->stitcher.updateOccupancyTree(pcloud);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pcloud);
  this->reference_path.Initialize(kdtree, this->params.map_buffer);

  //publish a downsampled version for faster visualization in RViz
  sensor_msgs::PointCloud2 map_ros;

  //downsample for visualization
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(pcloud);
  if(pcloud->points.size() > 250000){
    voxel_filter.setLeafSize(0.25f, 0.25f, 0.25f); //office downsample
  }
  else{
    voxel_filter.setLeafSize(0.5f, 0.5f, 0.5f); // perlin downsample
  }
  voxel_filter.filter(*downsampled);

  pcl::toROSMsg(*downsampled, map_ros);

  map_ros.header.frame_id = this->frame;
  pub_cloud.publish(map_ros);
}

Motion_Primitives::RosNode::LocalPlanner::~LocalPlanner() {
  // TODO
}

void Motion_Primitives::RosNode::LocalPlanner::LocalPlanner::timerCallback(
    const ros::TimerEvent &e) {
  if (this->abort) {
    Stop();
  }
}

void Motion_Primitives::RosNode::LocalPlanner::Start() {
  Motion_Primitives::primitiveParams time_params;
  time_params.map_buffer =  this->params.map_buffer;
  time_params.verbose = this->params.verbose;
  time_params.v_max = this->params.v_max;
  time_params.a_max = this->params.a_max;
  time_params.j_max = this->params.j_max;
  time_params.map_ub = this->params.map_ub;
  time_params.map_lb = this->params.map_lb;
  time_params.rho = this->params.rho;
  time_params.vf_magnitude = this->params.vf_magnitude;
  time_params.vf_zenith = this->params.vf_zenith;
  time_params.vf_azimuth = this->params.vf_azimuth;
  time_params.collision_check = this->params.collision_check;
  time_params.thrust_max = this->params.thrust_max;
  time_params.thrust_min = this->params.thrust_min;
  time_params.omega_max = this->params.omega_max;
  time_params.theta_max = this->params.theta_max;
  time_params.mass = this->params.mass;
  time_params.grav = this->params.grav;
  time_params.arm_length = this->params.arm_length;
  time_params.drag = this->params.drag;
  time_params.motor_thrust_max = this->params.motor_thrust_max;
  time_params.motor_thrust_min = this->params.motor_thrust_min;
  time_params.Jxx = this->params.Jxx;
  time_params.Jyy = this->params.Jyy;
  time_params.Jzz = this->params.Jzz;

  this->heuristic_dp.Initialize(time_params);
  this->stitcher.Initialize(time_params);

  if (this->params.geometric_planner_type == "astar") {
    this->geometric_planner = std::make_unique<Astar>();
  }
  else if (this->params.geometric_planner_type == "jps") {
    this->geometric_planner = std::make_unique<Jps>();
  }
  else {
    ROS_INFO("geometric planner unspecified, default to jps");
    this->geometric_planner = std::make_unique<Jps>();
  }

  //update param upper and lower bound
  this->geometric_planner->Initialize(this->params.resolution,this->params.map_buffer);
  Eigen::MatrixXd graph_limits = this->geometric_planner->setOccupancyMap(this->cloud);
  //update map_ub and map_lb based on graph limits
  this->params.map_ub = {graph_limits(0,1), graph_limits(1,1), graph_limits(2,1)};
  this->params.map_lb = {graph_limits(0,0), graph_limits(1,0), graph_limits(2,0)};

  //set rosparam
  ros::param::set("~GeometricPlanner/map_ub", this->params.map_ub);
  ros::param::set("~GeometricPlanner/map_lb", this->params.map_lb);
}

void Motion_Primitives::RosNode::LocalPlanner::Stop() {
  ROS_WARN("Killing primitives local planner");
  ros::shutdown();
}

void Motion_Primitives::RosNode::LocalPlanner::targetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  if (start_goal_points.size() >= 2){
    start_goal_points.clear();
  }
  double default_z = 1.0;

  Eigen::Vector3d target;
  target << msg->pose.position.x, msg->pose.position.y, default_z;
  //collision check point
  if(this->stitcher.collisionCheckMap(target, this->params.map_buffer) > 0){
    ROS_WARN("Target point is in collision");
  }
  else{
    this->mtx.lock();
    this->start_goal_points.push_back(target);
    pubStartAndGoal(target, start_goal_points.size()-1);
    this->mtx.unlock();

  }

  //if two points are received, set the start and goal, publish them and run the planner
  if (start_goal_points.size() == 2){
    this->mtx.lock();
    this->pose = start_goal_points[0];
    this->global_goal = start_goal_points[1];
    this->mtx.unlock();

    ROS_INFO("----------------Primitives local planner initialized and running----------------");

    pubData();

    // toggle if want to save data to csv
    // saveStates(this->stitcher.final_primitives, "stitcher_state_plot.csv");

  } 
  return;
}

void Motion_Primitives::RosNode::LocalPlanner::pubStartAndGoal(Eigen::Vector3d target, int id){
  visualization_msgs::Marker node;
  node.header.frame_id = this->frame;
  node.header.stamp = ros::Time::now();

  node.type = visualization_msgs::Marker::SPHERE;
  node.scale.x = 0.7;
  node.scale.y = 0.7;
  node.scale.z = 0.7;
  node.pose.position.x = target(0);
  node.pose.position.y = target(1);
  node.pose.position.z = target(2);
  node.pose.orientation.w = 1.;
  node.pose.orientation.x = 0.;
  node.pose.orientation.y = 0.;
  node.pose.orientation.z = 0.;
  if (id == 0){
    node.color.r = 1;
    node.color.g = 0;
    node.color.b = 0;
  }
  else{
    node.color.r = 0;
    node.color.g = 1;
    node.color.b = 0;
  }
  node.color.a = 0.8;
  node.action = visualization_msgs::Marker::ADD;
  node.id = id;

  this->marker_array.markers.push_back(node);

  pub_state.publish(this->marker_array);

}


void Motion_Primitives::RosNode::LocalPlanner::pubData(){

  pubRefPath(this->pose, this->global_goal);
  pubWaypoints(this->waypoints);

  //note STITCHER runs its own internal Heuristic DP, the following is just for RViz visualization
  pubTrajectory(this->heuristic_dp, this->pub_heuristic_dp_traj, "Heuristic DP", false);  
  pubAllDPTrajOptions();
 
  // pubTrajectory(this->stitcher, this->pub_stitcher_traj, "Stitcher");
  pubColoredTrajectory(this->stitcher, this->pub_colored_stitcher_traj, "Stitcher");
  calcLQMTJ(this->stitcher.final_primitives, "Stitcher");
  pubBubbles(this->stitcher);
}

void Motion_Primitives::RosNode::LocalPlanner::pubAllDPTrajOptions(){
  ROS_INFO("Publishing all DP options");
  std::unordered_map<int, Motion_Primitives::HeuristicDP::Node*> closed_set = this->heuristic_dp.getClosedSet();
  visualization_msgs::MarkerArray marker_arr;

  //make sure previous are deleted
  visualization_msgs::Marker kill_all;
  kill_all.action = visualization_msgs::Marker::DELETEALL;
  kill_all.header.frame_id = this->frame;
  kill_all.header.stamp = ros::Time::now();
  marker_arr.markers.push_back(kill_all);

  int counter = 0;
  // publish all the edges for each node
  for (int i=0; i< closed_set.size(); i++){
    Motion_Primitives::HeuristicDP::Node* dp_node = closed_set[i];
    for (int j=0; j< dp_node->adjacent_edges.size(); j++){
      Motion_Primitives::HeuristicDP::Edge* dp_edge = dp_node->adjacent_edges[j];
      pubPrimitive(dp_edge->primitive, marker_arr, counter);
    }
  }
  this->pub_dp_prim_options.publish(marker_arr);
}


template <typename PrimitiveType>
bool Motion_Primitives::RosNode::LocalPlanner::pubPrimitive(const PrimitiveType &primitive,
                                                            visualization_msgs::MarkerArray &marker_array,
                                                            int &counter){
  //using marker array 
  std::vector<Eigen::Vector3d> path;
  path.push_back(primitive.getPos(0));
  double dt = 0.05;

  for (int i = 1; i < ceil(primitive.horizon/dt); i++){
    // if (counter > 100) {
    //   break;
    // }
    path.push_back(primitive.getPos(i*dt));
    visualization_msgs::Marker line_segment;
    line_segment.header.frame_id = this->frame;
    line_segment.header.stamp = ros::Time::now();
    line_segment.action = visualization_msgs::Marker::ADD;
    line_segment.pose.orientation.w = 1.0;
    line_segment.id = counter;  // Each segment gets a unique ID
    line_segment.type = visualization_msgs::Marker::LINE_STRIP;
    line_segment.scale.x = 0.01;  // Line width

    geometry_msgs::Point p1, p2;
    p1.x = path[i-1](0);
    p1.y = path[i-1](1);
    p1.z = path[i-1](2);
    p2.x = path[i](0);
    p2.y = path[i](1);
    p2.z = path[i](2);

    line_segment.points.push_back(p1);
    line_segment.points.push_back(p2);
    line_segment.color.a = 0.7;
    line_segment.color.r = 255.0/255;
    line_segment.color.g = 255.0/255;
    line_segment.color.b = 255.0/255;

    marker_array.markers.push_back(line_segment);
    counter++;
  }
  return true;
}

template <typename PlannerType>
bool Motion_Primitives::RosNode::LocalPlanner::pubTrajectory(PlannerType &planner, 
                                                             ros::Publisher &publisher,
                                                             const std::string &planner_name,
                                                             bool verbose){
  if (!this->got_reference_path) {
    ROS_WARN("No reference path received yet");
    return false;
  }

  if(verbose){
    ROS_INFO("Generating %s trajectory", planner_name.c_str());
  }

  std::vector<Eigen::Vector3d> path;
  Eigen::MatrixXd start_state = Eigen::MatrixXd::Zero(4,3);
  start_state.row(0) = this->pose;
  
  //planner implement here
  planner.planPath(start_state,this->global_goal,this->waypoints);
  path = planner.final_path;

  if (verbose) {
    ROS_INFO("%s Trajectory Planning time: %f ", planner_name.c_str(), planner.planning_time);
    ROS_INFO("%s Trajectory Execution time: %f ", planner_name.c_str(), planner.execution_time);
  }

  nav_msgs::Path ros_path;
  ros_path.header.stamp = ros::Time::now();
  ros_path.header.frame_id = this->frame;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = this->frame;
  pose.pose.orientation.w = 1;

  for (int i = 0; i<path.size(); i++) {
    pose.pose.position.x = path[i](0);
    pose.pose.position.y = path[i](1);
    pose.pose.position.z = path[i](2);
    ros_path.poses.push_back(pose);
  }

  publisher.publish(ros_path);
  return true;
}

template <typename PlannerType>
bool Motion_Primitives::RosNode::LocalPlanner::pubColoredTrajectory(PlannerType &planner, 
                                                             ros::Publisher &publisher,
                                                             const std::string &planner_name){
  ROS_INFO("Generating %s trajectory", planner_name.c_str());
  if (!this->got_reference_path) {
    ROS_WARN("No reference path received yet");
    return false;
  }

  Eigen::MatrixXd start_state = Eigen::MatrixXd::Zero(4,3);
  start_state.row(0) = this->pose;
  
  //planner implement here
  planner.planPath(start_state,this->global_goal,this->waypoints);

  ROS_INFO("%s Trajectory Planning time: %f ", planner_name.c_str(), planner.planning_time);
  ROS_INFO("%s Trajectory Execution time: %f ", planner_name.c_str(), planner.execution_time);

  std::vector<Eigen::Vector3d> path;
  std::vector<double> vel_mag;
  getStates(planner.final_primitives, path, vel_mag);

  visualization_msgs::MarkerArray marker_array;

  //make sure previous are deleted
  visualization_msgs::Marker kill_all;
  kill_all.action = visualization_msgs::Marker::DELETEALL;
  kill_all.header.frame_id = this->frame;
  kill_all.header.stamp = ros::Time::now();
  marker_array.markers.push_back(kill_all);

  double max_vel = this->params.v_max;

  double min_vel = 0.0;
  for (int i = 0; i< path.size()-1; i++){
    visualization_msgs::Marker line_segment;
    line_segment.header.frame_id = this->frame;
    line_segment.header.stamp = ros::Time::now();
    line_segment.action = visualization_msgs::Marker::ADD;
    line_segment.pose.orientation.w = 1.0;
    line_segment.id = i;  // Each segment gets a unique ID
    line_segment.type = visualization_msgs::Marker::LINE_STRIP;
    line_segment.scale.x = 0.3;  // Line width

    geometry_msgs::Point p1, p2;
    p1.x = path[i](0);
    p1.y = path[i](1);
    p1.z = path[i](2);
    p2.x = path[i+1](0);
    p2.y = path[i+1](1);
    p2.z = path[i+1](2);

    std_msgs::ColorRGBA color = getColorForVelocity(vel_mag[i], min_vel, max_vel);
    std_msgs::ColorRGBA color2 = getColorForVelocity(vel_mag[i+1], min_vel, max_vel);

    line_segment.points.push_back(p1);
    line_segment.points.push_back(p2);
    line_segment.colors.push_back(color);
    line_segment.colors.push_back(color2);

    marker_array.markers.push_back(line_segment);
  }

  publisher.publish(marker_array);
  return true;
}

std_msgs::ColorRGBA Motion_Primitives::RosNode::LocalPlanner::getColorForVelocity(double velocity, double v_min, double v_max) {
  std_msgs::ColorRGBA color;

  double ratio = (velocity - v_min) / (v_max - v_min);
  ratio = std::min(1.0, std::max(0.0, ratio));  // Clamp ratio between 0 and 1

  double r = std::max(0.0, std::min(1.0, 1.5 - fabs(4.0 * ratio - 3.0)));
  double g = std::max(0.0, std::min(1.0, 1.5 - fabs(4.0 * ratio - 2.0)));
  double b = std::max(0.0, std::min(1.0, 1.5 - fabs(4.0 * ratio - 1.0)));

  color.r = r;
  color.g = g;
  color.b = b;

  color.a = 1.0;  

  return color;
}

template <typename PrimitiveType>
void Motion_Primitives::RosNode::LocalPlanner::getStates(const std::vector<PrimitiveType> &primitives,
                                                         std::vector<Eigen::Vector3d> &positions,
                                                         std::vector<double> &vel_mags){
  std::vector<PrimitiveType> final_primitives = primitives;
  std::reverse(final_primitives.begin(), final_primitives.end());

  for (int i = 0; i<final_primitives.size(); i++) {
    for (double j = 0; j <final_primitives[i].horizon; j+= 0.01){
      positions.push_back(final_primitives[i].getPos(j));
      vel_mags.push_back(final_primitives[i].getVel(j).norm());
    }
  }                                              
}

void Motion_Primitives::RosNode::LocalPlanner::pubRefPath(Eigen::Vector3d start, Eigen::Vector3d goal){
  ROS_INFO("Generating reference path");
  //timing geometric planner
  auto start_time = std::chrono::high_resolution_clock::now();
  this->geometric_planner->plan(start,goal);
  if (this->geometric_planner->path.size() == 0){
    ROS_WARN("Geometric planner failed to find a path");
    this->got_reference_path = false;
    return;
  }
  this->reference_path.SetMaxDistVertices(this->params.max_dist_btwn_wps);
  
  if (this->params.geometric_planner_type == "jps") {
    this->reference_path.ProcessPath(this->geometric_planner->path,"jps");
  }
  else {
    this->reference_path.ProcessPath(this->geometric_planner->path,"dense");
  }

  this->waypoints = this->reference_path.waypoints;
  std::vector<Eigen::Vector3d> path = this->waypoints;
  this->got_reference_path = true;

  ROS_INFO("Geometric Planner Waypoint Generation Planning time: %f", std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count());

  nav_msgs::Path ros_path;
  ros_path.header.stamp = ros::Time::now();
  ros_path.header.frame_id = this->frame;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = this->frame;
  pose.pose.orientation.w = 1;

  for (int i = 0; i<path.size(); i++) {
    pose.pose.position.x = path[i](0);
    pose.pose.position.y = path[i](1);
    pose.pose.position.z = path[i](2);
    ros_path.poses.push_back(pose);
  }

  this->pub_ref_path.publish(ros_path);
}

void Motion_Primitives::RosNode::LocalPlanner::pubWaypoints(std::vector<Eigen::Vector3d> waypoints){ 
  visualization_msgs::MarkerArray marker_arr;

  //make sure previous are deleted
  visualization_msgs::Marker kill_all;
  kill_all.action = visualization_msgs::Marker::DELETEALL;
  kill_all.header.frame_id = this->frame;
  kill_all.header.stamp = ros::Time::now();
  marker_arr.markers.push_back(kill_all);

  visualization_msgs::Marker node;
  node.header.frame_id = this->frame;
  node.header.stamp = ros::Time::now();

  for (int i = 1; i < waypoints.size()-1; i++){
    node.type = visualization_msgs::Marker::SPHERE;
    node.scale.x = 0.2;
    node.scale.y = 0.2;
    node.scale.z = 0.2;
    node.pose.position.x = waypoints[i](0);
    node.pose.position.y = waypoints[i](1);
    node.pose.position.z = waypoints[i](2);
    node.pose.orientation.w = 1.;
    node.pose.orientation.x = 0.;
    node.pose.orientation.y = 0.;
    node.pose.orientation.z = 0.;

    node.color.r = 239./255.;
    node.color.g = 41./255.;
    node.color.b = 41./255.;
  
    node.color.a = 0.8;
    node.action = visualization_msgs::Marker::ADD;
    node.id = i;

    marker_arr.markers.push_back(node);
  }
  pub_waypoints.publish(marker_arr);
}

template <typename PlannerType>
void Motion_Primitives::RosNode::LocalPlanner::pubBubbles(PlannerType &planner){
  ROS_INFO("Generating prev known safe bubbles");
  
  std::vector<Eigen::MatrixXd> bubbles = planner.safe_bubble_map;
  visualization_msgs::MarkerArray marker_arr;

  //make sure previous are deleted
  visualization_msgs::Marker kill_all;
  kill_all.action = visualization_msgs::Marker::DELETEALL;
  kill_all.header.frame_id = this->frame;
  kill_all.header.stamp = ros::Time::now();
  marker_arr.markers.push_back(kill_all);

  visualization_msgs::Marker node;
  node.header.frame_id = this->frame;
  node.header.stamp = ros::Time::now();

  int count = 0;
  for (int i = 0; i < bubbles.size(); i++){
    for (int j = 0; j < bubbles[i].rows(); j++){
      node.type = visualization_msgs::Marker::SPHERE;
      node.scale.x = 2*bubbles[i](j,3);
      node.scale.y = 2*bubbles[i](j,3);
      node.scale.z = 2*bubbles[i](j,3);
      node.pose.position.x = bubbles[i](j,0);
      node.pose.position.y = bubbles[i](j,1);
      node.pose.position.z = bubbles[i](j,2);
      node.pose.orientation.w = 1.;
      node.pose.orientation.x = 0.;
      node.pose.orientation.y = 0.;
      node.pose.orientation.z = 0.;

      node.color.r = 255./255.;
      node.color.g = 199./255.;
      node.color.b = 44./255.;

      node.color.a = 0.15;
      node.action = visualization_msgs::Marker::ADD;
      node.id = count;
      count++;

      marker_arr.markers.push_back(node);
    }
  }
  pub_bubbles.publish(marker_arr);

}

template<typename PrimitiveType>
double Motion_Primitives::RosNode::LocalPlanner::calcLQMTJ(const std::vector<PrimitiveType> &primitives, std::string planner_name){
  std::vector<PrimitiveType> final_primitives = primitives;
  double Jcost = 0.;

  for(int i = 0; i < final_primitives.size(); i++){
    Jcost += final_primitives[i].calcJ(this->params.rho);
  }

  ROS_INFO("%s total cost J: %f ", planner_name.c_str(), Jcost);

  return Jcost;
}

template<typename PrimitiveType>
void Motion_Primitives::RosNode::LocalPlanner::saveStates(const std::vector<PrimitiveType> &primitives,
                                                          const std::string &file_name,
                                                          bool reverse){
  std::vector<PrimitiveType> final_primitives = primitives;
  if (reverse){
    std::reverse(final_primitives.begin(), final_primitives.end());
  }
  //reverse to plot from start to end
  double cost = 0.;
  double dt = 0.01;
  Eigen::VectorXd position, velocity, acceleration, jerk;

  //initializing file
  std::string package_name = "stitcher_planner";
  std::string relative_path = "/data/" + file_name;

  std::ofstream myfile(ros::package::getPath(package_name) + relative_path);
  if(!myfile.is_open()){
    std::cout << "Error opening file" << std::endl;
    return;
  }
  myfile << "time, px, py, pz, vx, vy, vz, ax, ay, az, jx, jy, jz, new_segment\n";

  double time = 0.0;
  for(int i = 0; i < final_primitives.size(); i++){
    for(double t = 0; t < final_primitives[i].horizon; t+=dt){
      time += dt;
      position = final_primitives[i].getPos(t);
      velocity = final_primitives[i].getVel(t);
      acceleration = final_primitives[i].getAccel(t);
      jerk = final_primitives[i].getJerk(t);

      myfile << time << "," << position(0) << "," << position(1) << "," << position(2);
      myfile << "," << velocity(0) << "," << velocity(1) << "," << velocity(2);
      myfile << "," << acceleration(0) << "," << acceleration(1) << "," << acceleration(2);
      myfile << "," << jerk(0) << "," << jerk(1) << "," << jerk(2) << "," << (t==0) << std::endl;
    }
  }
  myfile.close();
  std::cout << "Generated file " << file_name << " successfully!" << std::endl;
}

void Motion_Primitives::RosNode::LocalPlanner::getParams() {
  ros::param::param<double>("~Primitives/v_max", this->params.v_max, 5.);
  ros::param::param<double>("~Primitives/a_max", this->params.a_max, 2.);
  ros::param::param<double>("~Primitives/j_max", this->params.j_max, 2.);
  ros::param::param<double>("~Primitives/rho", this->params.rho, 500.);
  ros::param::param<std::vector<double>>("~Primitives/vf_zenith", this->params.vf_zenith, {});
  ros::param::param<std::vector<double>>("~Primitives/vf_magnitude", this->params.vf_magnitude, {});
  ros::param::param<std::vector<double>>("~Primitives/vf_azimuth", this->params.vf_azimuth, {});


  ros::param::param<double>("~Planner/map_collision_buffer", this->params.map_buffer, 0.5);
  ros::param::param<bool>("~Planner/collision_check", this->params.collision_check, false);
  ros::param::param<bool>("~Planner/verbose", this->params.verbose, false);

  ros::param::param<std::string>("~GeometricPlanner/geometric_planner_type", this->params.geometric_planner_type, "jps");
  ros::param::param<double>("~GeometricPlanner/resolution", this->params.resolution, 1);
  ros::param::param<std::vector<double>>("~GeometricPlanner/map_ub", this->params.map_ub, {});
  ros::param::param<std::vector<double>>("~GeometricPlanner/map_lb", this->params.map_lb, {});
  ros::param::param<double>("~GeometricPlanner/max_dist_btwn_wps", this->params.max_dist_btwn_wps, 1000.0);

  ros::param::param<double>("~Robot/thrust_max", this->params.thrust_max, 12.);
  ros::param::param<double>("~Robot/thrust_min", this->params.thrust_min, 0.);
  ros::param::param<double>("~Robot/omega_max", this->params.omega_max, 6.);
  ros::param::param<double>("~Robot/theta_max", this->params.theta_max, 0.6);
  ros::param::param<double>("~Robot/mass", this->params.mass, 0.5);
  ros::param::param<double>("~Robot/grav", this->params.grav, 9.8);
  ros::param::param<double>("~Robot/motor_thrust_max", this->params.motor_thrust_max, 3.);
  ros::param::param<double>("~Robot/motor_thrust_min", this->params.motor_thrust_min, 0.);
  ros::param::param<double>("~Robot/arm_length", this->params.arm_length, 0.15);
  ros::param::param<double>("~Robot/drag", this->params.drag, 0.2);
  ros::param::param<double>("~Robot/Jxx", this->params.Jxx, 0.1);
  ros::param::param<double>("~Robot/Jyy", this->params.Jyy, 0.15);
  ros::param::param<double>("~Robot/Jzz", this->params.Jzz, 0.1);

  // Convert from degs to rads
  for (int i = 0; i < this->params.vf_zenith.size(); i++) {
    this->params.vf_zenith[i] *= 3.14159 / 180.;
  } 
  for (int i = 0; i < this->params.vf_azimuth.size(); i++) {
    this->params.vf_azimuth[i] *= 3.14159 / 180.;
  } 

}