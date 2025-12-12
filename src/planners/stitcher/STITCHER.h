#ifndef STITCHER_H
#define STITCHER_H

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <unordered_map>
#include <chrono>
#include <atomic>
#include <HeuristicDP.h>
#include <CollisionChecker.h>
#include <FeasibilityChecker.h>
#include <primitiveParams.h>

namespace Motion_Primitives {
    template <typename MotionPrimitiveType>
    class STITCHER;
}

template <typename MotionPrimitiveType>
class Motion_Primitives::STITCHER
    : public Motion_Primitives::CollisionChecker, public Motion_Primitives::FeasibilityChecker {
public:
    /**
     * Constructor
     **/
    STITCHER();
    void planPath(Eigen::MatrixXd start_state, Eigen::VectorXd goal, std::vector<Eigen::Vector3d> & reference_waypoints);
    void Initialize(const Motion_Primitives::primitiveParams params);
                    
    /**
     * Destructor
     */
    virtual ~STITCHER();

    Motion_Primitives::primitiveParams params;
    std::vector<Eigen::Vector3d> final_path;
    std::vector<MotionPrimitiveType> final_primitives;
    std::vector<Eigen::Vector3d> waypoints;
    Eigen::VectorXd goal;

    float execution_time;
    float planning_time;
    std::atomic<bool> replan;
    std::vector<Eigen::MatrixXd> safe_bubble_map;
    int node_unconnected;

protected:
    struct Node{
        Eigen::MatrixXd state = Eigen::MatrixXd::Zero(4,3);
        double g_cost = std::numeric_limits<double>::infinity();
        double h_cost = 0.;
        int waypoint_id = -1;
        int node_id = -1;
        int node_at_wp_id = -1;
        Node* parent_ptr = nullptr; //pointer to parent node
        MotionPrimitiveType primitive; //primitive connecting this node with its parent
    };

    struct CompareCost{
        bool operator()(const Node* n1, const Node* n2) {
            return (n1->g_cost+n1->h_cost) > (n2->g_cost+n2->h_cost);
        }
    };
    
    void Reinitialize(int num_waypoints);
    int getNumNodes(int num_waypoints, int num_vel_opts, std::vector<int> &num_nodes_at_id);
    int calcNodeID(int waypoint_id, int wp_node_id, const std::vector<int> &num_nodes_at_id);
    void getSuccessors(Node *current_node,
                       std::priority_queue<Node*, std::vector<Node*>, CompareCost> &open_set,
                       std::unordered_map<int, Node*> &open_map,
                       std::unordered_map<int, Node*> &closed_set);
    virtual bool generatePrimitive(MotionPrimitiveType &primitive, 
                                   const Eigen::MatrixXd &current_state, 
                                   const Eigen::MatrixXd &des_state) = 0;         
    void buildEndStateMenu(int num_waypoints, std::vector<std::vector<Eigen::Vector3d>> &vf_states);
    double collisionCheckWithBubbles(MotionPrimitiveType &primitive, int &waypoint_id);
    void calcFinalPrimitives(Node &goal_node, std::vector<MotionPrimitiveType> &final_primitives);
    void dispStateDirection(const Node &current);
    Eigen::MatrixXd getRotationMatrix(Eigen::Vector3d);
    void calcFinalPath(const std::vector<MotionPrimitiveType> &final_primitives, 
                       std::vector<Eigen::Vector3d> &final_path);
    
    std::vector<std::vector<Eigen::MatrixXd>> end_states;
    Motion_Primitives::HeuristicDP heuristic_dp{false}; //not saving path

    //counters
    int invalid_counter;
    int global_counter;
    int collision_count;
    int bubble_counter;
    Eigen::VectorXd safe_bubble_map_generated;

    //timers
    float bubble_collision_check_time;
    float feasibility_check_time;
    
    //node management
    std::vector<int> num_nodes_at_id;
    int num_nodes;

    bool verbose;
};

#include "STITCHER.tpp"

#endif