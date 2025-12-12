#pragma once

#include <Eigen/Dense>
#include <queue>
#include <vector>
#include <cmath>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class GeometricPlanner {
    public:
        GeometricPlanner();
        virtual ~GeometricPlanner();
        virtual void plan(Eigen::Vector3d start, Eigen::Vector3d goal) = 0;
        virtual void Initialize(const double resolution, const double robot_radius) = 0;
        Eigen::MatrixXd setOccupancyMap(const pcl::PointCloud<pcl::PointXYZ> &cloud);
        std::vector<Eigen::Vector3d> path = {};

        struct Node {
            double x, y, z;
            double dx, dy, dz;
            double g_cost;
            double h_cost;
            int id;
            Node * parent;
        };

        struct CompareCost{
            bool operator()(const Node *n1, const Node *n2) {
                return (n1->g_cost + n1->h_cost) > (n2->g_cost + n2->h_cost);
            }
        };

    protected:
        void setObs(const double x, const double y, const double z);
        virtual void getSuccessors(Node* current_node, 
                           std::priority_queue<Node*, std::vector<Node*>, CompareCost> &open, 
                           std::unordered_map<int,Node*> &open_ptrs, 
                           std::unordered_map<int, Node*> &closed) = 0;
        std::vector<Eigen::Vector3d> getFinalPath();
        bool verifyNode(Node node);
        bool verifyNode(double x, double y, double z);
        double calcHeuristic(Node node1, Node node2);
        int calcXYZIndex(double pt, int coord_plane);
        void setNodeIndex(Node *node);
        int coordToId(double x, double y, double z);

        Eigen::MatrixXd graph_lim; //[x_lb, x_ub]; [y_lb, y_ub]; [z_lb, z_ub]
        Eigen::Vector3i graph_size;
        double resolution;
        double map_buffer;
        Eigen::VectorXi occupancy_map;
        Node start_node;
        Node goal_node;

};