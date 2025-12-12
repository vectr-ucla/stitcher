#ifndef JPS3D_H
#define JPS3D_H

#include "geometric_planner/GeometricPlanner.h"

struct JPS3DNeib {
    // for each (dx,dy,dz) these contain:
    //    ns: neighbors that are always added
    //    f1: forced neighbors to check
    //    f2: neighbors to add if f1 is forced
    double ns[27][3][26];
    double f1[27][3][12];
    double f2[27][3][12];
    double resolution;

    // nsz contains the number of neighbors for the four different types of moves:
    // no move (norm 0):        26 neighbors always added
    //                          0 forced neighbors to check (never happens)
    //                          0 neighbors to add if forced (never happens)
    // straight (norm 1):       1 neighbor always added
    //                          8 forced neighbors to check
    //                          8 neighbors to add if forced
    // diagonal (norm sqrt(2)): 3 neighbors always added
    //                          8 forced neighbors to check
    //                          12 neighbors to add if forced
    // diagonal (norm sqrt(3)): 7 neighbors always added
    //                          6 forced neighbors to check
    //                          12 neighbors to add if forced
    int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
    JPS3DNeib();
    void setResolution();
    private:
    void Neib(double dx, double dy, double dz, int norm1, int dev, double& tx, double& ty, double& tz);
    void FNeib(double dx, double dy, double dz, int norm1, int dev,
        double& fx, double& fy, double& fz,
        double& nx, double& ny, double& nz);
};

class Jps : public GeometricPlanner {
    public:
        Jps();
        ~Jps();
        void plan(Eigen::Vector3d start, Eigen::Vector3d goal);
        void Initialize(const double resolution, const double robot_radius);
        
    private:
        void getSuccessors(Jps::Node* curr, 
                        std::priority_queue<Jps::Node*, std::vector<Jps::Node*>, Jps::CompareCost> &open_set, 
                        std::unordered_map<int, Jps::Node*> &open_set_ptrs, 
                        std::unordered_map<int, Jps::Node*> &closed_set);

        bool jump(double x, double y, double z, double dx, double dy, double dz, double& new_x, double& new_y, double& new_z);
        bool hasForced(double x, double y, double z, double dx, double dy, double dz);

        std::vector<std::vector<int>> ns_;
        std::shared_ptr<JPS3DNeib> jn3d_;
};


#endif // JPS3D_H