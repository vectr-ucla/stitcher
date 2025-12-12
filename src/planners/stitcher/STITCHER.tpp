#include "STITCHER.h"
#include <iostream>

template <typename MotionPrimitiveType>
Motion_Primitives::STITCHER<MotionPrimitiveType>::STITCHER(): FeasibilityChecker(0.1) {
    this->replan = false;
    this->verbose = false;
}

template <typename MotionPrimitiveType>
Motion_Primitives::STITCHER<MotionPrimitiveType>::~STITCHER() {
  // ROS deletes stuff automatically
}

template <typename MotionPrimitiveType> 
void Motion_Primitives::STITCHER<MotionPrimitiveType>::Initialize(const Motion_Primitives::primitiveParams params) 
{
  //function initializing the planner 
  this->params = params;
  this->verbose = params.verbose;
  //path properties init
  this->final_path = {}; 
  this->final_primitives = {};
  this->waypoints = {};
  this->end_states = {};

  //timers
  this->planning_time = 0.;
  this->execution_time = 0.;
  this->bubble_collision_check_time = 0.;
  this->feasibility_check_time = 0.;

  //counters
  this->global_counter = 0; // global num of prims
  this->invalid_counter = 0; // invalid num of prims
  this->collision_count = 0; // collision prims
  this->bubble_counter = 0; // how many safe bubbles created

  // bubble collision init
  this->safe_bubble_map = {};

  // reconnect graph params reset
  this->node_unconnected = -1; // which node was unconnected

  this->heuristic_dp.Initialize(params);
  setParams(params); //setting params for FeasibilityChecker
}

template <typename MotionPrimitiveType> 
void Motion_Primitives::STITCHER<MotionPrimitiveType>::Reinitialize(int num_waypoints) 
{
  //function resetting all shared variables in the planner (important for replanning nodes)
  //path properties reset
  this->final_path.clear();
  this->final_primitives.clear();
  this->waypoints.clear();
  this->end_states.clear();

  this->waypoints.reserve(num_waypoints);
  this->end_states.reserve(num_waypoints);
  this->final_primitives.reserve(num_waypoints-1);

  //timers
  this->planning_time = 0.;
  this->execution_time = 0.;
  this->bubble_collision_check_time = 0.;
  this->feasibility_check_time = 0.;

  //counters
  this->global_counter = 0; // global num of prims
  this->invalid_counter = 0; // invalid num of prims
  this->collision_count = 0; // collision prims
  this->bubble_counter = 0; // how many safe bubbles created

  // bubble collision reset
  this->safe_bubble_map.clear();
  this->safe_bubble_map_generated = Eigen::VectorXd::Zero(num_waypoints);
  this->safe_bubble_map.reserve(num_waypoints);
  this->safe_bubble_map.resize(num_waypoints);

  // reconnect graph params reset
  this->node_unconnected = -1; // which node was unconnected

}

template <typename MotionPrimitiveType>
int Motion_Primitives::STITCHER<MotionPrimitiveType>::getNumNodes(int num_waypoints, int num_vel_opts, std::vector<int> &num_nodes_at_id){
  int num_nodes = 0;
  num_nodes_at_id.clear();
  
  switch(num_waypoints){
    case 1:
      std::cout << "Not enough waypoints" << std::endl;
      break;
    default:
      num_nodes_at_id.push_back(1);
      for (int i = 1; i <= num_waypoints-2; i++){
        num_nodes_at_id.push_back(num_vel_opts);
      }
      num_nodes_at_id.push_back(1);
  }
  num_nodes = (num_waypoints-2)*num_vel_opts + 2;
   
  return num_nodes;
}

template <typename MotionPrimitiveType>
int Motion_Primitives::STITCHER<MotionPrimitiveType>::calcNodeID(int waypoint_id, int wp_node_id, const std::vector<int> &num_nodes_at_id){
  int node_id = 0;
  for (int i = 0; i < waypoint_id; i++){
    node_id += num_nodes_at_id[i];
  }
  node_id += wp_node_id;
  return node_id;
}

template <typename MotionPrimitiveType>
void Motion_Primitives::STITCHER<MotionPrimitiveType>::buildEndStateMenu(int num_waypoints, std::vector<std::vector<Eigen::Vector3d>> &vf_states){
  this->end_states.resize(num_waypoints);

  //end states only have velocity info
  Eigen::MatrixXd final_state = Eigen::MatrixXd::Zero(2,3); 
  for (int i = 0; i < num_waypoints; i++){
    for (int j = 0; j < vf_states[i].size(); j++){
      final_state.row(0) = this->waypoints[i];
      final_state.row(1) = vf_states[i][j];
      this->end_states[i].push_back(final_state);
    }
  }
}

template <typename MotionPrimitiveType>
void Motion_Primitives::STITCHER<MotionPrimitiveType>::planPath(Eigen::MatrixXd start_state, Eigen::VectorXd goal, std::vector<Eigen::Vector3d> & reference_waypoints) {
  auto begin = std::chrono::high_resolution_clock::now();
  
  int num_waypoints = reference_waypoints.size();
  if(this->verbose){std::cout << "Number of waypoints: " << num_waypoints << std::endl;}
  Reinitialize(num_waypoints);

  //save goal and waypoints
  this->goal = goal;
  this->waypoints = reference_waypoints;
  
  int num_vel_opts = this->params.vf_zenith.size()*this->params.vf_azimuth.size()*this->params.vf_magnitude.size() + 1;
  
  int num_nodes = getNumNodes(num_waypoints, num_vel_opts, this->num_nodes_at_id);
  this->num_nodes = num_nodes;

  // defining sets
  std::priority_queue<Node*, std::vector<Node*>, CompareCost> open_set;
  std::unordered_map<int, Node*> open_map;
  std::unordered_map<int, Node*> closed_set;

  //run dynamic programming to build map
  this->heuristic_dp.planPath(start_state, goal, reference_waypoints);
  
  buildEndStateMenu(num_waypoints, this->heuristic_dp.vf_states);

  //initialize start node
  Node start_node;
  start_node.state = start_state;
  start_node.waypoint_id = 0;
  start_node.node_id = 0;
  start_node.node_at_wp_id = 0;
  start_node.parent_ptr = nullptr;
  start_node.g_cost = 0.;
  start_node.h_cost = this->params.rho*this->heuristic_dp.cost_map[0][0];

  //initializations of sets and nodes
  open_set.push(&start_node);
  open_map[0] = &start_node;

  bool goal_reached = false;
  Node* current = nullptr;
  Node* goal_ptr = nullptr;

  while(!goal_reached && !open_set.empty()){
    current = open_set.top();
    open_set.pop();

    if (open_map[current->node_id] != current) {
      continue; // Skip invalid (outdated) node
    }

    //remove from open, add to closed
    open_map.erase(current->node_id);
    closed_set.insert({current->node_id,current});

    //check if goal reached
    if (current->waypoint_id == num_waypoints-1 && current->g_cost != std::numeric_limits<double>::infinity()){
      goal_reached = true;
      goal_ptr = current;
      break; 
    } 

    //add valid successors to open set
    getSuccessors(current, open_set, open_map, closed_set);
  }

  if (!goal_reached){
    std::cout << "ERROR: No motion primitive path found" << std::endl;
    this->node_unconnected = current->waypoint_id;

    //look in closed set for best path up to current waypoint
    for (auto it = closed_set.begin(); it != closed_set.end(); ++it){
      if (it->second->waypoint_id == current->waypoint_id && it->second->g_cost + it->second->h_cost < current->g_cost + current->h_cost){
        current = it->second;
      }
    }
    std::cout << "Best motion primitive path found up to waypoint: " << current->waypoint_id << std::endl;
    goal_ptr = current;
   
  }
  else{
    std::cout << "Motion Primitive Goal Reached" << std::endl;
  }

  //go backwards to generate final primitives and path
  calcFinalPrimitives(*goal_ptr, this->final_primitives);
  calcFinalPath(this->final_primitives, this->final_path); 

  this->planning_time = std::chrono::duration<float>(std::chrono::high_resolution_clock::now() - begin).count();

  if(this->verbose){
  double total_edges = (num_waypoints-3)*pow(num_vel_opts,2) + 2*num_vel_opts;
  std::cout << "----- STITCHER Statistics -----\n";
  //graph analysis (nodes and edges)
  std::cout << "Number of edges(primitives) generated: " << this->global_counter << "/" << total_edges << "\n";
  std::cout << "Number of nodes explored: " << closed_set.size() << "/" << num_nodes << "\n";

  //number of primitives in collision and invalid 
  std::cout << "Number of pruned in-collision edges(primitives): " << this->collision_count << "\n";
  std::cout << "Number of pruned invalid edges(primitives): " << this->invalid_counter << " \n";
  std::cout << "Number of edges(primitives) outside of known safe bubbles (vanilla collision checking req): " << this->bubble_counter << "\n";

  //timing 
  std::cout << "Time to get heuristic cost-to-go map (Stage 2 DP): " << this->heuristic_dp.planning_time << std::endl;
  std::cout << "Time to collision check: " << this->bubble_collision_check_time << std::endl;
  std::cout << "Time to do state/actuator feasibility check: " << this->feasibility_check_time << std::endl;
  std::cout << "Time to do motion primitive graph search: " << this->planning_time - this->heuristic_dp.planning_time - this->bubble_collision_check_time - this->feasibility_check_time << std::endl;
  std::cout << "Total planning time: " << this->planning_time << std::endl;
  std::cout << "-------------------------------\n";
  }

  //deallocate all nodes 
  for (auto &node_pair : open_map) {
    delete node_pair.second;
  }

  closed_set.clear();
  open_map.clear();
}

template <typename MotionPrimitiveType>
void Motion_Primitives::STITCHER<MotionPrimitiveType>::getSuccessors(Node *current_node,
                                                                           std::priority_queue<Node*, std::vector<Node*>, CompareCost> &open_set,
                                                                           std::unordered_map<int, Node*> &open_map,
                                                                           std::unordered_map<int, Node*> &closed_set) {
  int num_primitives = this->end_states[current_node->waypoint_id+1].size();
  int node_at_wp_id, node_id;

  for (int i = 0; i < num_primitives; i++){
    node_at_wp_id = 0; 
    node_id = 0;
    if (current_node->waypoint_id == this->waypoints.size()-2){
      node_at_wp_id = 0;
      node_id = this->num_nodes-1;
    }
    else{
      node_at_wp_id = i;
      node_id = calcNodeID(current_node->waypoint_id+1, node_at_wp_id, this->num_nodes_at_id);
    }
    
    Eigen::MatrixXd des_state = this->end_states[current_node->waypoint_id+1][i];

    //check if in closed set already
    if (closed_set.count(node_id) != 0){
      continue;
    }
  
    MotionPrimitiveType primitive;
    this->global_counter++;
    if (!generatePrimitive(primitive, current_node->state, des_state)){
      this->invalid_counter++;
      continue;
    }

    //vanilla collision check (no safe bubbles)
    // if(this->params.collision_check){
    //     double collisions = collisionCheckMap(primitive, this->params.map_buffer, this->params.v_max);

    //     if (collisions > 0) {
    //         continue;
    //     }
    // }

    //collision checking (reusing calculated safe bubbles if available)
    if (this->params.collision_check){
      double collisions = collisionCheckWithBubbles(primitive, current_node->waypoint_id);

      if (collisions > 0) {
        this->collision_count++;
        continue;
      }
    }

    double cost_so_far = current_node->g_cost + primitive.getCost();

    Node* successor = (open_map.count(node_id) != 0) ? open_map[node_id] : nullptr;
    if (successor == nullptr){
      //create new node
      successor = new Node();
      successor->node_id = node_id;
      successor->waypoint_id = current_node->waypoint_id+1;
      successor->node_at_wp_id = node_at_wp_id;
      successor->state = primitive.getState(primitive.horizon);
      successor->g_cost = cost_so_far;
      successor->h_cost = this->params.rho*this->heuristic_dp.cost_map[successor->waypoint_id][i];
      successor->parent_ptr = current_node;
      successor->primitive = primitive;
      //add to open set
      open_set.push(successor);
      open_map[node_id] = successor;
    }

    if(cost_so_far < successor->g_cost){
      //update successor
      successor->state = primitive.getState(primitive.horizon);
      successor->g_cost = cost_so_far;
      successor->parent_ptr = current_node;
      successor->primitive = primitive;
      //add to open set
      open_set.push(successor);
      open_map[node_id] = successor;
    }
  }
}

template <typename MotionPrimitiveType>
double Motion_Primitives::STITCHER<MotionPrimitiveType>::collisionCheckWithBubbles(MotionPrimitiveType &primitive, int &waypoint_id){
  auto begin_bubble_collision = std::chrono::high_resolution_clock::now();
  double collisions = 0;

  if (this->safe_bubble_map_generated(waypoint_id) == 0){
    //generate safe bubble map
    std::vector<Eigen::Vector4d> safe_bubble_data; //(cx,cy,cz,r)

    //collision check and store the known safe bubbles
    collisions = collisionCheckMap(primitive, this->params.map_buffer, safe_bubble_data, this->params.v_max);

    //data management
    Eigen::MatrixXd safe_bubble_mat(safe_bubble_data.size(),4);
    for (int j = 0; j < safe_bubble_data.size(); j++){
      safe_bubble_mat.row(j) << safe_bubble_data[j].transpose();
    }

    //store in waypoint pair -> bubbles map
    this->safe_bubble_map[waypoint_id] = safe_bubble_mat;
    this->safe_bubble_map_generated(waypoint_id) = 1;
  } 
  else{
    //use safe bubble map for collision checking
    Eigen::MatrixXd safe_bubble_centers = this->safe_bubble_map[waypoint_id].leftCols(3);
    Eigen::VectorXd safe_bubble_radii = this->safe_bubble_map[waypoint_id].col(3);

    double t = 0.;
    bool check_horizon = false;
    double last_time = 0.;
    while (t <= primitive.horizon && !check_horizon){
      if (t == primitive.horizon){
        check_horizon = true; //check one last time at horizon
      }

      Eigen::Vector3d pos = primitive.getPos(t);
      Eigen::VectorXd dist_to_centers = (safe_bubble_centers.rowwise() - pos.transpose()).rowwise().norm();
  
      int min_index;
      double circle_check = (dist_to_centers.array() - (safe_bubble_radii.array()-this->params.map_buffer)).minCoeff(&min_index);
      double min_dist = dist_to_centers(min_index);

      //if outside safe bubble, do vanilla collision check 
      if(circle_check > 0){  
        //append new spheres
        std::vector<Eigen::Vector4d> safe_bubble_data;

        //collision check and store the known safe bubbles
        collisions = collisionCheckMap(primitive, this->params.map_buffer, safe_bubble_data, this->params.v_max);

        //data management
        int prev_num_bubbles = this->safe_bubble_map[waypoint_id].rows();
        Eigen::MatrixXd new_safe_bubble_data(prev_num_bubbles + safe_bubble_data.size(), 4);
        new_safe_bubble_data << this->safe_bubble_map[waypoint_id], Eigen::MatrixXd::Zero(safe_bubble_data.size(), 4);

        for (int j = 0; j < safe_bubble_data.size(); j++){
          new_safe_bubble_data.row(prev_num_bubbles + j) << safe_bubble_data[j].transpose();
        }
        this->safe_bubble_map[waypoint_id] = new_safe_bubble_data;

        this->bubble_counter++; //count how many times we had to do vanilla collision check
        break;
      }

      //next time sample
      last_time = t;
      t += ((safe_bubble_radii(min_index)-this->params.map_buffer)-min_dist)/this->params.v_max;

      if (t - last_time < 0.001){
        t = t + 0.0005;
      }

      //if t greater than horizon, set t to horizon
      if (t > primitive.horizon){
        t = primitive.horizon;
      }
    }

  }
  auto end_bubble_collision = std::chrono::high_resolution_clock::now();
  typedef std::chrono::duration<float> float_seconds;
  this->bubble_collision_check_time += std::chrono::duration_cast<float_seconds>(end_bubble_collision-begin_bubble_collision).count();

  return collisions;
}

template <typename MotionPrimitiveType>
void Motion_Primitives::STITCHER<MotionPrimitiveType>::calcFinalPrimitives(Node &goal_node, std::vector<MotionPrimitiveType> &final_primitives) {
  //calculates final primitives by backtracking from goal node
  //this->execution_time = 0.;
  Node current = goal_node;
  final_primitives.push_back(current.primitive);

  while (current.parent_ptr != nullptr) {
    //displaying final primitive info 
    if(this->verbose){
      std::cout << "--------Primitive ending at waypoint " << current.waypoint_id << "--------\n";
      dispStateDirection(current);
    }
    
    //adding execution and iterating through parents
    this->execution_time += current.primitive.horizon;
    current = *current.parent_ptr;
    final_primitives.push_back(current.primitive);
  }

  if(this->verbose){
    //No primitive ending at waypoint 0 (since start state)
    std::cout << "--------Primitive ending at waypoint 0--------\n";
    std::cout << "Zero velocity\n";
    std::cout << "Zero acceleration\n";
    std::cout << "-------------------------------\n";
  }

  //removing primitive ending at prim 0 (since start state)
  final_primitives.erase(final_primitives.end());
}

template <typename MotionPrimitiveType>
void Motion_Primitives::STITCHER<MotionPrimitiveType>::dispStateDirection(const Node &current) {
  //calculates and displays the direction of velocity and acceleration wrt the plane normal (normal vector to plane of symm at waypoint)
  double norm = current.state.row(1).norm();
  double accel_norm = current.state.row(2).norm();
  Eigen::Vector3d plane_normal = this->heuristic_dp.reference_path.plane_normal_vectors[current.waypoint_id-1];
  Eigen::Vector3d original_vel = getRotationMatrix(plane_normal).inverse()*current.state.row(1).transpose();
  Eigen::Vector3d original_accel = getRotationMatrix(plane_normal).inverse()*current.state.row(2).transpose();

  std::cout << "Time horizon: " << current.primitive.horizon << std::endl;
  
  if (norm < 1e-3){
    std::cout << "Zero velocity\n";
  }
  else{
    std::cout << "Velocity norm: " << norm << ", ";
    std::cout << "zenith: " << std::acos(original_vel(2)/norm)*180/3.14159 << ", ";
    std::cout << "azimuth: " << std::atan2(original_vel(1), original_vel(0))*180/3.14159 << "\n"; 
  }

  if (accel_norm < 1e-3){
    std::cout << "Zero acceleration\n";
  }
  else{
    std::cout << "Acceleration norm: " << accel_norm << ", ";
    std::cout << "zenith: " << std::acos(original_accel(2)/accel_norm)*180/3.14159 << ", ";
    std::cout << "azimuth: " << std::atan2(original_accel(1), original_accel(0))*180/3.14159 << "\n"; 
  }
}

template <typename MotionPrimitiveType>
Eigen::MatrixXd Motion_Primitives::STITCHER<MotionPrimitiveType>::getRotationMatrix(Eigen::Vector3d frame_vector) {
  double pitch = -atan2(frame_vector(2), sqrt(pow(frame_vector(0),2)+pow(frame_vector(1),2))); //pitch
  double yaw = atan2(frame_vector(1), frame_vector(0)); //yaw
  Eigen::Matrix3d R, Ry, Rz;

  Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;
  Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
  R = Rz*Ry;
  return R;
}

//calculate final path
template <typename MotionPrimitiveType>
void Motion_Primitives::STITCHER<MotionPrimitiveType>::calcFinalPath(const std::vector<MotionPrimitiveType> &final_primitives, std::vector<Eigen::Vector3d> &final_path) {
  for (int i = 0; i < final_primitives.size(); i++) {
    for (double t = final_primitives[i].horizon; t > 0; t -= 0.1)
      final_path.push_back(final_primitives[i].getPos(t));
  }
}
