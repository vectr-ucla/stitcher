#ifndef ASTAR_H
#define ASTAR_H

#include "geometric_planner/GeometricPlanner.h"

class Astar : public GeometricPlanner{
    public:
        Astar();
        ~Astar();
        void plan(Eigen::Vector3d start, Eigen::Vector3d goal);
        void Initialize(const double resolution, const double robot_radius);
         
    private:
        // void setObs(const double x, const double y, const double z);
        void getSuccessors(Node* current_node, 
                           std::priority_queue<Node*, std::vector<Node*>, CompareCost> &open, 
                           std::unordered_map<int,Node*> &open_ptrs, 
                           std::unordered_map<int, Node*> &closed);
        
        Eigen::MatrixXd getMotions();
};

#endif // ASTAR_H