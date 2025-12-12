
#ifndef REFERENCEPATH_H
#define REFERENCEPATH_H

#include <Eigen/Dense>
#include <vector>
#include <CollisionChecker.h>
#include <pcl/kdtree/kdtree_flann.h>

class ReferencePath : public Motion_Primitives::CollisionChecker{
    public:

    std::vector<Eigen::Vector3d> waypoints; // sparse waypoints of the path
    std::vector<Eigen::Vector3d> segment_vectors; // unit vectors of each segment of the path
    Eigen::MatrixXd segment_headings; // heading angles of the path 
    Eigen::VectorXd segment_lengths; // lengths of each segment of the path
    std::vector<Eigen::Vector3d> plane_normal_vectors; // normal vector to separating hyperplane at each waypoint junction

    ReferencePath();
    virtual ~ReferencePath();
    void Initialize(const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, 
                    const double collision_buffer);
    void SetMaxDistVertices(double max_dist);
    void ProcessPath(const std::vector<Eigen::Vector3d> &graph_search_path, std::string path_type);
    
    private:
    std::vector<Eigen::Vector3d> getWaypoints(const std::vector<Eigen::Vector3d> &graph_search_path,
                                              std::string path_type);
    std::vector<Eigen::Vector3d> sparsifyPath(const std::vector<Eigen::Vector3d> &dense_path);
    std::vector<Eigen::Vector3d> addPointsToPath(const std::vector<Eigen::Vector3d> &sparse_path);
    void getSegmentInformation(std::vector<Eigen::Vector3d> waypoints);
    void getPlaneNormalVectors(std::vector<Eigen::Vector3d> waypoints);
    double calcLineCost(Eigen::Vector3d point1, Eigen::Vector3d point2);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    double collision_buffer;
    bool initialized;
    double max_dist_btwn_wps; // max distance between waypoints

};

#endif // REFERENCEPATH_H