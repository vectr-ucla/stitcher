#include "HeuristicDP.h"
#include <iostream>

Motion_Primitives::HeuristicDP::HeuristicDP(bool save_path) {
  // Constructor
  this->save_path = save_path; //if turned on save final primitives and path
}

Motion_Primitives::HeuristicDP::~HeuristicDP() {
  // ROS deletes stuff automatically
}

void Motion_Primitives::HeuristicDP::Initialize(const Motion_Primitives::primitiveParams params) 
{
  //function initializing the planner 
  this->params = params;
  this->final_path = {};
  this->final_primitives = {};
  this->waypoints = {};
  this->vf_states = {};
  this->planning_time = 0.;
  this->execution_time = 0.;
  this->goal_vel = Eigen::VectorXd::Zero(3);
  this->primitive_counter = 0;
  this->cost_map = {};
  this->closed_set.clear();
}

void Motion_Primitives::HeuristicDP::buildVfStateMenu(Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel){
  this->vf_states.clear();
  this->vf_states.reserve(this->waypoints.size());
  //start waypoint at start vel
  this->vf_states.push_back({start_vel});
  this->cost_map.push_back({0.0});
  for (int i = 1; i < this->waypoints.size()-1; i++){
    this->vf_states.push_back(getDesVelocity(i));
    std::vector<double> costs(this->vf_states[i].size(), 0.0);
    this->cost_map.push_back(costs);
  }
  //for last waypoint end at rest or goal velocity
  this->vf_states.push_back({goal_vel});
  this->cost_map.push_back({0.0});
}

void Motion_Primitives::HeuristicDP::planPath(Eigen::MatrixXd start_state, 
                                              Eigen::VectorXd goal, 
                                              std::vector<Eigen::Vector3d> & reference_waypoints) {
  auto begin = std::chrono::high_resolution_clock::now();
   
  // empty paths
  this->final_path = {};
  this->final_primitives = {};
  this->planning_time = 0.;
  this->execution_time = 0.;
  this->primitive_counter = 0;
  this->closed_set.clear();
  this->cost_map.clear();

  //initialize w waypoints
  this->reference_path.ProcessPath(reference_waypoints, "sparse");

  this->waypoints = reference_waypoints;
  int num_waypoints = reference_waypoints.size();
  buildVfStateMenu(start_state.row(1),this->goal_vel);
  
  // defining sets
  std::vector<Node*> open_set;
  open_set.reserve((num_waypoints-2)*this->vf_states[1].size()+2);
  
  //initialize start node
  Node * start_node = nullptr;
 
  //initialize goal point
  Eigen::MatrixXd goal_state = Eigen::MatrixXd::Zero(4,3);
  goal_state.row(0) = goal;
  goal_state.row(1) = this->goal_vel;
  Node* goal_node = new Node(goal_state);
  goal_node->waypoint_id = num_waypoints-1;
  goal_node->node_id = (num_waypoints-2)*this->vf_states[1].size() + 1;
  goal_node->g_cost = 0;

  open_set.push_back(goal_node);
  
  bool start_reached = false;
  Node* current = nullptr;

  while(!open_set.empty()){
    auto current_it = open_set.begin();
    current = *current_it;

    //finding minimum cost node
    for (auto it = open_set.begin(); it != open_set.end(); it++) {
      auto node = *it;
      if (node->g_cost <= current->g_cost) {
          current = node;
          current_it = it;
      }
    }

    this->closed_set.insert({current->node_id,current});
    open_set.erase(current_it);

    // check if goal reached
    if (current->waypoint_id == 0){
      start_reached = true;
      start_node = current;
      continue; //need to continue instead of break to ensure whole tree is built
    } 

    //add valid successors to open set
    getNeighbors(current, open_set, this->closed_set);
  }

  if (!start_reached) {
    std::cout << "No Stage 2 Path found" << std::endl;
  }
  else{
    std::cout << "Stage 2 cost-to-go compilation completed" << std::endl;
  }

  //saving final primitives and path
  if(this->save_path) {
    calcFinalPrimitives(start_node, this->final_primitives);
    calcFinalPath(this->final_primitives, this->final_path);
  }

  this->execution_time = start_node->g_cost;
  auto end = std::chrono::high_resolution_clock::now();
  typedef std::chrono::duration<float> float_seconds;
  float duration = std::chrono::duration_cast<float_seconds>(end-begin).count();
  this->planning_time = duration;

  //deallocate all nodes in open set
  for (auto node : open_set) {
    delete node;
  }
  open_set.clear();
}

Motion_Primitives::HeuristicDP::Node* Motion_Primitives::HeuristicDP::findNodeInSet(std::vector<Node*> &set, int node_id){
  for (int i = 0; i < set.size(); i++){
    if (set[i]->node_id == node_id){
      return set[i];
    }
  }
  return nullptr;
}

void Motion_Primitives::HeuristicDP::getNeighbors(Node * const current_node,
                                                  std::vector<Node*> &open_set,
                                                  std::unordered_map<int, Node*> &closed_set) {     
                                                          
  std::vector<Eigen::Vector3d> prev_vel_options = this->vf_states[current_node->waypoint_id-1];
  Eigen::VectorXd prev_waypoint = this->waypoints[current_node->waypoint_id-1];

  int num_primitives = prev_vel_options.size();

  for (int i = 0; i < num_primitives; i++){
    this->primitive_counter++;

    Eigen::MatrixXd prev_state = Eigen::MatrixXd::Zero(4,3);
    prev_state.row(0) = prev_waypoint;
    prev_state.row(1) = prev_vel_options[i];

    //generating an edge
    Motion_Primitives::MinTimePrimitive primitive;
    primitive.genTraj(prev_state, current_node->state, this->params.a_max, this->params.az_max);
    primitive.setCost(primitive.horizon);

    Edge * edge = new Edge(primitive);

    double cost_so_far = current_node->g_cost + primitive.getCost();
    //labeling nodes and waypoints in forward direction
    int waypoint_id = current_node->waypoint_id - 1;
    int node_id = (waypoint_id == 0) ? 0 : 1 + (waypoint_id-1)*num_primitives + i;
    edge->edge_id = {node_id, current_node->node_id};
    edge->wp_id = {waypoint_id, current_node->waypoint_id};
    edge->cost_to_go = current_node->g_cost;

    //if in closed set, continue
    if (closed_set.count(node_id) != 0){
      closed_set[node_id]->adjacent_edges.push_back(edge);
      continue;
    }
    
    //find if neighbor is already in the open set
    Node * neighbor = findNodeInSet(open_set, node_id);
    if (neighbor == nullptr){
      //create new node if it's not in the open_set
      neighbor = new Node(primitive.getState(0));
      neighbor->waypoint_id = waypoint_id;
      neighbor->node_id = node_id;
      neighbor->g_cost = cost_so_far;
      neighbor->child_ptr = current_node;
      neighbor->best_primitive = primitive;
      open_set.push_back(neighbor);
      neighbor->adjacent_edges.push_back(edge);
      this->cost_map[waypoint_id][i] = cost_so_far;
    }
    // if neighbor is in the open set already, add the edge to the edge set
    else{
      if(cost_so_far < neighbor->g_cost){
          //update successor
          neighbor->state = primitive.getState(0);
          neighbor->best_primitive = primitive;
          neighbor->child_ptr = current_node;
          neighbor->g_cost = cost_so_far;
          this->cost_map[waypoint_id][i] = cost_so_far;
      }

      neighbor->adjacent_edges.push_back(edge);
    }
  }

}

Eigen::MatrixXd Motion_Primitives::HeuristicDP::getRotationMatrix(Eigen::Vector3d frame_vector) {
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

Eigen::Vector3d Motion_Primitives::HeuristicDP::sphericalToCartesian(double zenith, double azimuth, double radius){
  Eigen::Vector3d cart_coordinate;
  cart_coordinate(0) = radius * sin(zenith)*cos(azimuth);
  cart_coordinate(1) = radius * sin(zenith)*sin(azimuth);
  cart_coordinate(2) = radius * cos(zenith);
  return cart_coordinate;
}

std::vector<Eigen::Vector3d> Motion_Primitives::HeuristicDP::getDesVelocity(int waypoint_id) {
  //generate all velocity headings 
 
  std::vector<Eigen::Vector3d> des_vel;
  //getting normal vector to the plane of symmetry at waypoint
  Eigen::VectorXd plane_normal = this->reference_path.plane_normal_vectors[waypoint_id - 1]; // waypoint id - 1 = plane normal idx
  Eigen::Vector3d velocity;
  double weight;
  for (int i = 0; i < this->params.vf_magnitude.size(); i++){
    weight = this->params.vf_magnitude[i]*this->params.v_max;
    for (int j = 0; j < this->params.vf_zenith.size(); j++){
      for (int k = 0; k < this->params.vf_azimuth.size(); k++){
        //if ref vec
        if(this->params.vf_zenith[j] > 2*3.14){
          velocity = getRotationMatrix(this->reference_path.segment_vectors[waypoint_id])*sphericalToCartesian(90*3.1415/180., this->params.vf_azimuth[k], weight);
        }
        else{
          //add normal vector rotation
          velocity = getRotationMatrix(plane_normal)*sphericalToCartesian(this->params.vf_zenith[j], this->params.vf_azimuth[k], weight);
        }
        
        des_vel.push_back(velocity);
      }
    }
  }

  des_vel.push_back(Eigen::VectorXd::Zero(3)); 
 
  return des_vel;
}

void Motion_Primitives::HeuristicDP::calcFinalPrimitives(Node * start_node, std::vector<Motion_Primitives::MinTimePrimitive> &final_primitives) {
  
  Node * current = start_node;
  final_primitives.push_back(current->best_primitive);
  while (current->child_ptr != nullptr) {
    current = current->child_ptr;
    final_primitives.push_back(current->best_primitive);
  }

  final_primitives.erase(final_primitives.end());
}

//calculate final path
void Motion_Primitives::HeuristicDP::calcFinalPath(std::vector<Motion_Primitives::MinTimePrimitive> final_primitives, std::vector<Eigen::Vector3d> &final_path) {
  for (int i = 0; i < final_primitives.size(); i++) {
    for (double t = 0; t < final_primitives[i].horizon; t+= 0.1)
      final_path.push_back(final_primitives[i].getPos(t));
  }
}

std::unordered_map<int, Motion_Primitives::HeuristicDP::Node*> Motion_Primitives::HeuristicDP::getClosedSet() {
  return this->closed_set;
}