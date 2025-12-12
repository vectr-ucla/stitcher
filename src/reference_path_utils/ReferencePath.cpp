#include "ReferencePath.h"
#include <iostream>

ReferencePath::ReferencePath(){
    this->max_dist_btwn_wps = 1000.; // max distance between waypoints
}

void ReferencePath::Initialize(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, double collision_buffer){

    this->kdtree = kdtree;
    this->collision_buffer = collision_buffer;
    this->initialized = true;
}

void ReferencePath::SetMaxDistVertices(double max_dist){
    // function to set max distance between waypoints
    this->max_dist_btwn_wps = max_dist;
}

void ReferencePath::ProcessPath(const std::vector<Eigen::Vector3d> &graph_search_path, std::string path_type){
    if (path_type == "dense") { //for planners like A* and Dijkstra which give dense paths full of node connections
        this->waypoints = getWaypoints(graph_search_path, path_type);
    }
    else if (path_type == "jps") { // for planners like JPS which give sparse paths with only turning points
        this->waypoints = getWaypoints(graph_search_path, path_type);
    }
    else if (path_type == "sparse") { // for direct waypoints
        this->waypoints = graph_search_path;
    }
    else {
        std::cout << "Invalid path type. Please enter 'dense' or 'sparse'." << std::endl;
    }
    
    int num_segments = this->waypoints.size()-1;
    
    this->segment_vectors.resize(num_segments);
    this->segment_headings = Eigen::MatrixXd::Zero(2,num_segments);
    this->segment_lengths = Eigen::VectorXd::Zero(num_segments);

    getSegmentInformation(this->waypoints);
    getPlaneNormalVectors(this->segment_vectors);
}


ReferencePath::~ReferencePath(){} //ROS does this automatically

std::vector<Eigen::Vector3d> ReferencePath::getWaypoints(const std::vector<Eigen::Vector3d> &graph_search_path, std::string path_type){
    //function to get waypoints from graph search path
    std::vector<Eigen::Vector3d> search_path;
    search_path = graph_search_path;
    //graph algorithms give path from goal to start, so reverse it
    std::reverse(search_path.begin(), search_path.end());

    if (path_type == "jps") {
        std::vector<Eigen::Vector3d> dense_path = addPointsToPath(search_path);
        return sparsifyPath(dense_path);
    }

    //sparsify the path
    return sparsifyPath(search_path);
}

std::vector<Eigen::Vector3d> ReferencePath::addPointsToPath(const std::vector<Eigen::Vector3d> &sparse_path){
    // add points such that no two consecutive points are farther apart than max_dist_btwn_wps
    std::vector<Eigen::Vector3d> dense_path;
    for (int i = 0; i < sparse_path.size()-1;i++){

        Eigen::Vector3d dir = sparse_path[i+1] - sparse_path[i];
        double dist_btwn_pts = dir.norm();
        if (dist_btwn_pts < 1e-9) {
            dense_path.push_back(sparse_path[i]);
            continue; //skip if points are too close
        }
        dir = dir/dist_btwn_pts; //normalize

        int num_addtl_pts = std::floor(dist_btwn_pts/this->max_dist_btwn_wps);
        dense_path.push_back(sparse_path[i]);

        double spacing = dist_btwn_pts/(num_addtl_pts + 1); //make sure points are evenly spaced
        for (int j = 1; j <= num_addtl_pts; j++) {
            dense_path.push_back(sparse_path[i] + j * spacing * dir);
        }
    }
    dense_path.push_back(sparse_path.back());
    return dense_path;
}

std::vector<Eigen::Vector3d> ReferencePath::sparsifyPath(const std::vector<Eigen::Vector3d> &dense_path){
    if(!this->initialized){
        std::cerr << "ERROR: Please Initialize Reference Path with a KD Tree before continuing" << std::endl;
    }

    // function to sparsify a dense path by removing nodes that cause jagged edges 
    int num_nodes = dense_path.size();
    Eigen::Vector3d dense_goal;
    std::vector<Eigen::Vector3d> sparse_path = {};
    double epsilon = 1e-10;

    for (int i=0; i<num_nodes; i++){
        dense_goal = dense_path[i];

        if (sparse_path.size() >= 2){

            //if adding the new point (dense goal) to the path is cheaper than replacing the last pt (there's a collision if you remove the last pt) then add dense goal
            if (calcLineCost(sparse_path[sparse_path.size()-2], sparse_path[sparse_path.size()-1]) + 
                calcLineCost(sparse_path[sparse_path.size()-1], dense_goal) < 
                calcLineCost(sparse_path[sparse_path.size()-2], dense_goal)-epsilon ){

                sparse_path.push_back(dense_goal);
            }
            else{
                sparse_path[sparse_path.size()-1] = dense_goal;
            }
        }
        else {
            sparse_path.push_back(dense_goal);
        }
    }

    return sparse_path;
}

double ReferencePath::calcLineCost(Eigen::Vector3d point1, Eigen::Vector3d point2){
    // function to calculate cost of line between two points

    //checking collision 
    if (collisionLineCheck(point1, point2, this->collision_buffer, this->kdtree)> 0){
        return std::numeric_limits<double>::infinity();
    }
    //check if line is too long
    if ((point2 - point1).norm() > this->max_dist_btwn_wps){
        return std::numeric_limits<double>::infinity();
    }
    return (point2 - point1).norm();
}

void ReferencePath::getSegmentInformation(std::vector<Eigen::Vector3d> waypoints){
    // function to get unit vectors of each segment of the path

    int num_segments = waypoints.size()-1;

    Eigen::Vector3d segment_vector;

    for (int i=0; i<num_segments; i++){
        //calculating segment length
        segment_vector = waypoints[i+1] - waypoints[i];
        this->segment_lengths[i] = segment_vector.norm();

        // unit vector of segment
        segment_vector = segment_vector / segment_vector.norm();
        this->segment_vectors[i] = segment_vector;
        // heading angle of segment
        this->segment_headings(0,i) = -atan2(segment_vector(2), sqrt(pow(segment_vector(0),2)+pow(segment_vector(1),2))); //pitch
        this->segment_headings(1,i) = atan2(segment_vector(1), segment_vector(0)); //yaw
        
    }
}

void ReferencePath::getPlaneNormalVectors(std::vector<Eigen::Vector3d> segment_vectors){
    // function to get normal vector to separating hyperplane at each waypoint junction

    int num_planes = segment_vectors.size()-1;
    this->plane_normal_vectors.resize(num_planes);

    for (int i=0; i<num_planes; i++){
        Eigen::Vector3d plane_normal = segment_vectors[i+1] + segment_vectors[i]; 
        plane_normal = plane_normal/plane_normal.norm();
        
        this->plane_normal_vectors[i] = plane_normal;
    }
}



