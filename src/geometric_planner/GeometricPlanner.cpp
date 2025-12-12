#include "GeometricPlanner.h"
#include <iostream>

GeometricPlanner::GeometricPlanner() {}

GeometricPlanner::~GeometricPlanner() {}


Eigen::MatrixXd GeometricPlanner::setOccupancyMap(const pcl::PointCloud<pcl::PointXYZ> &cloud){
    if (cloud.empty()) {
        std::cout << "GeometricPlanner::setOccupancyMap — ERROR: received EMPTY cloud!" << std::endl;
    }
    //determine bounds

    //set graph lims to first point 
    this->graph_lim.col(0) << cloud.points[0].x, cloud.points[0].y, cloud.points[0].z;
    this->graph_lim.col(1) << cloud.points[0].x, cloud.points[0].y, cloud.points[0].z;

    //iterate through point cloud to get limits
    for (int i = 0; i < cloud.points.size(); i++){
        if (cloud.points[i].x < this->graph_lim(0,0)){
            this->graph_lim(0,0) = cloud.points[i].x;
        }
        if (cloud.points[i].x > this->graph_lim(0,1)){
            this->graph_lim(0,1) = cloud.points[i].x;
        }
        if (cloud.points[i].y < this->graph_lim(1,0)){
            this->graph_lim(1,0) = cloud.points[i].y;
        }
        if (cloud.points[i].y > this->graph_lim(1,1)){
            this->graph_lim(1,1) = cloud.points[i].y;
        }
        if (cloud.points[i].z < this->graph_lim(2,0)){
            this->graph_lim(2,0) = cloud.points[i].z;
        }
        if (cloud.points[i].z > this->graph_lim(2,1)){
            this->graph_lim(2,1) = cloud.points[i].z;
        }
    }

    std::cout << "Graph Limits: " << this->graph_lim << std::endl;

    //determine size of occupancy map
    this->graph_size = ((this->graph_lim.col(1) - this->graph_lim.col(0))/this->resolution).cast<int>();
    this->graph_size = this->graph_size.array() + 1; // add 1 to each dimension to account for 0 index
    
    this->occupancy_map = Eigen::VectorXi::Zero(this->graph_size.prod()); // prod of x*y*z size

    //set obstacles
    for (int i = 0; i < cloud.points.size(); i++){
        setObs(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    }
    std::cout<< "Occupancy Map Set!" << std::endl;

    return this->graph_lim;
}

void GeometricPlanner::setObs(const double x, const double y, const double z){
    // set obstacles
    int idx_x = calcXYZIndex(x,0);
    int idx_y = calcXYZIndex(y,1);
    int idx_z = calcXYZIndex(z,2);

    int id = coordToId(x, y, z);

    // set occupancy map grid cell to 1 if obstacle present
    this->occupancy_map(id) = 1;

    // Calculate buffer in terms of grid cells
    int buffer_cells = ceil(this->map_buffer / this->resolution); // Assuming resolution is known

    // Add cells surrounding obstacle depending on map buffer
    for (int i = -buffer_cells; i <= buffer_cells; i++) {
        for (int j = -buffer_cells; j <= buffer_cells; j++) {
            for (int k = -buffer_cells; k <= buffer_cells; k++) {
                int new_idx_x = idx_x + i;
                int new_idx_y = idx_y + j;
                int new_idx_z = idx_z + k;

                // Skip if indices are out of bounds
                if (new_idx_x < 0 || new_idx_x >= this->graph_size(0) ||
                    new_idx_y < 0 || new_idx_y >= this->graph_size(1) ||
                    new_idx_z < 0 || new_idx_z >= this->graph_size(2)) {
                    continue;
                }

                int new_id = new_idx_x + new_idx_y * this->graph_size(0) + new_idx_z * this->graph_size(0) * this->graph_size(1);
                this->occupancy_map(new_id) = 1;
            }
        }
    }
}

std::vector<Eigen::Vector3d> GeometricPlanner::getFinalPath() {
    
    // code to get final course
    std::vector<Eigen::Vector3d> path;
    GeometricPlanner::Node current = this->goal_node;
    Eigen::Vector3d pt;
    pt(0) = current.x;
    pt(1) = current.y;
    pt(2) = current.z;

    path.push_back(pt);
    while (current.parent != nullptr){
        current = *current.parent;
        pt(0) = current.x;
        pt(1) = current.y;
        pt(2) = current.z;

        path.push_back(pt);
    }
    return path;
}


bool GeometricPlanner::verifyNode(GeometricPlanner::Node node) {
    //verify if node is in the map & not in collision
    if (node.x < graph_lim(0,0) || node.x > graph_lim(0,1) || 
        node.y < graph_lim(1,0) || node.y > graph_lim(1,1) || 
        node.z < graph_lim(2,0) || node.z > graph_lim(2,1)) {
        return false;
    }

    if (this->occupancy_map(node.id) == 1) {
        return false;
    }

    return true;
}  


bool GeometricPlanner::verifyNode(double x, double y, double z) {
  //verify if node is in the map & not in collision
  if (x < graph_lim(0,0) || x > graph_lim(0,1) || 
      y < graph_lim(1,0) || y > graph_lim(1,1) || 
      z < graph_lim(2,0) || z > graph_lim(2,1)) {
      return false;
  }
  
  int id = coordToId(x, y, z);
  if (this->occupancy_map(id) == 1) {
        return false;
  }

  return true;
}

double GeometricPlanner::calcHeuristic(GeometricPlanner::Node node1, GeometricPlanner::Node node2) {
    // calculate heuristic
    return sqrt(pow(node1.x - node2.x, 2.0) + pow(node1.y - node2.y, 2.0) + pow(node1.z - node2.z, 2.0));
}

int GeometricPlanner::calcXYZIndex(double pt, int coord_plane){
    // calculate index in 3 coordinate planes of node
    double lower_bound = this->graph_lim.row(coord_plane)(0);

    int idx = static_cast<int>((pt - lower_bound)/this->resolution);

    return idx;
}

void GeometricPlanner::setNodeIndex(GeometricPlanner::Node* node) {
    // convert node to index
    node->id = coordToId(node->x, node->y, node->z);
}

int GeometricPlanner::coordToId(double x, double y, double z) {
  int idx_x = calcXYZIndex(x,0);
  int idx_y = calcXYZIndex(y,1);
  int idx_z = calcXYZIndex(z,2);
  int id = idx_x + idx_y*this->graph_size(0) + idx_z*this->graph_size(0)*this->graph_size(1);
  return id;
}

