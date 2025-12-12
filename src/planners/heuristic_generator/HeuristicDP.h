#ifndef HEURISTIC_DP_H
#define HEURISTIC_DP_H

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <MinTimePrimitive.h>
#include <ReferencePath.h>
#include <primitiveParams.h>

namespace Motion_Primitives {
    class HeuristicDP;
}

class Motion_Primitives::HeuristicDP {
public:
    struct Edge {
        std::pair<int, int> wp_id;
        std::pair<int, int> edge_id;
        double cost_to_go; //min cost to reach goal from end_node
        Motion_Primitives::MinTimePrimitive primitive;

        Edge(const Motion_Primitives::MinTimePrimitive& primitive)
            : wp_id({-1,-1}), 
            edge_id({-1,-1}),
            cost_to_go(0.),
            primitive(primitive) {}
    };
    
    struct Node{
        Eigen::MatrixXd state;
        double g_cost;
        int waypoint_id;
        int node_id;
        Node* child_ptr;
        Motion_Primitives::MinTimePrimitive best_primitive;
        std::vector<Edge*> adjacent_edges;

        Node(Eigen::MatrixXd state): state(state), g_cost(std::numeric_limits<double>::infinity()), waypoint_id(-1), node_id(-1), child_ptr(nullptr), best_primitive(), adjacent_edges({}) {}

    };
    /**
     * Constructor
     **/
    HeuristicDP(bool save_path = true);
    void planPath(Eigen::MatrixXd start_state, Eigen::VectorXd goal, std::vector<Eigen::Vector3d> & reference_waypoints);
    void Initialize(const Motion_Primitives::primitiveParams params);
    std::unordered_map<int, Node*> getClosedSet();
                    
    /**
     * Destructor
     */
    virtual ~HeuristicDP();

    Motion_Primitives::primitiveParams params;
    std::vector<Eigen::Vector3d> final_path;
    std::vector<Motion_Primitives::MinTimePrimitive> final_primitives;
    std::vector<Eigen::Vector3d> waypoints;
    std::vector<std::vector<double>> cost_map;

    float execution_time;
    float planning_time;
    Eigen::Vector3d goal_vel;
    ReferencePath reference_path;
    std::vector<std::vector<Eigen::Vector3d>> vf_states;

private:
    Node* findNodeInSet(std::vector<Node*> &set, int node_id);
    void getNeighbors(Node * const current_node,
                      std::vector<Node*> &open_set,
                      std::unordered_map<int, Node*> &closed_set); 
    void buildVfStateMenu(Eigen::Vector3d start_vel, Eigen::Vector3d goal_vel);
    std::vector<Eigen::Vector3d> getDesVelocity(int waypoint_id); 
    void calcFinalPrimitives(Node * start_node, 
                             std::vector<Motion_Primitives::MinTimePrimitive> &final_primitives);
    void calcFinalPath(std::vector<Motion_Primitives::MinTimePrimitive> final_primitives, 
                       std::vector<Eigen::Vector3d> &final_path);
    Eigen::Vector3d sphericalToCartesian(double zenith, double azimuth, double radius);
    Eigen::MatrixXd getRotationMatrix(Eigen::Vector3d);

    int primitive_counter;
    std::unordered_map<int, Node*> closed_set;
    bool save_path;

};

#endif