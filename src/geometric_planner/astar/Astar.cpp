// code base inspired by PythonRobotics Astar implementation

#include "Astar.h"
#include <iostream>

Astar::Astar(){
}

Astar::~Astar(){
    // ROS deletes stuff automatically
}

void Astar::Initialize(const double resolution, const double map_buffer){
    // get size of the map and bounds
    this->resolution = resolution; 
    this->map_buffer = map_buffer;
    this->graph_lim = Eigen::MatrixXd::Zero(3,2);
    this->path = {};
}

void Astar::plan(Eigen::Vector3d start, Eigen::Vector3d goal)
{
    // sort through best motions and develop path
    this->path.clear();

    this->start_node.x = start(0);
    this->start_node.y = start(1);
    this->start_node.z = start(2);
    setNodeIndex(&this->start_node);
    this->start_node.g_cost = 0.0; 
    this->start_node.parent = nullptr;

    this->goal_node.x = goal(0);
    this->goal_node.y = goal(1);
    this->goal_node.z = goal(2);
    setNodeIndex(&this->goal_node);
    this->goal_node.g_cost = 0.0;
    this->goal_node.h_cost = 0.0;
    this->goal_node.parent = nullptr;
    
    if (!verifyNode(this->start_node)){
        std::cout << "Start Node in collision!" << std::endl;
        return;
    }

    if (!verifyNode(this->goal_node)){
        std::cout << "Goal Node in collision!" << std::endl;
        return;
    }

    //calculate h_cost for start node
    this->start_node.h_cost = calcHeuristic(this->start_node, this->goal_node);

    //define open and closed set
    std::unordered_map<int, Node*> closed_set;
    std::priority_queue<Node*, std::vector<Node*>, CompareCost> open_set;
    std::unordered_map<int, Node*> open_set_ptrs;

    open_set.push(&this->start_node);
    open_set_ptrs.insert({this->start_node.id, &this->start_node});

    Node* current = nullptr;
    
    //while open set not empty
    while(open_set.size()>0){
        // get node with smallest cost node from open set
        current = open_set.top();
       
        open_set.pop();
        open_set_ptrs.erase(current->id);
        closed_set.insert({current->id,current});

        // check if goal node is reached
        if (current->id == this->goal_node.id){
            // goal reached
            std::cout << "Astar Goal Reached!" << std::endl;
            this->goal_node.parent = current->parent;
            break;
        }

        getSuccessors(current, open_set, open_set_ptrs, closed_set);
    }

    //goal not reached
    if (current->id != this->goal_node.id){
        std::cout << "No A* Path found!" << std::endl;
    }
    else{
        this->path = getFinalPath();
    }

    //deallocate memory used for open and closed sets
    for (auto& [id, ptr] : open_set_ptrs) {
        if (ptr != &this->start_node && ptr != &this->goal_node){
            delete ptr;
        }
    }
    for (auto& [id, ptr] : closed_set) {
        if (ptr != &this->start_node && ptr != &this->goal_node){
            delete ptr;
        }
    }
    open_set_ptrs.clear();
    closed_set.clear();
}

void Astar::getSuccessors(Node* current_node, 
                          std::priority_queue<Node*, std::vector<Node*>, Astar::CompareCost> &open_set, 
                          std::unordered_map<int, Node *> &open_set_ptrs, 
                          std::unordered_map<int, Node *> &closed_set){
    // get successors of node
    Eigen::MatrixXd motions = getMotions();
    for (int i = 0; i < motions.rows(); i++){
        Node * successor = new Node();
        successor->x = current_node->x + motions(i,0);
        successor->y = current_node->y + motions(i,1);
        successor->z = current_node->z + motions(i,2);
        successor->g_cost = current_node->g_cost + motions(i,3);
        successor->h_cost = calcHeuristic(*successor, this->goal_node);
        successor->parent = closed_set[current_node->id]; 
        setNodeIndex(successor);

        //check if in collision/in map
        if (!verifyNode(*successor)){
            // add to successors
            continue;
        }

        //check if this successor is in closed set
        if (closed_set.count(successor->id)){
            continue;
        }

        //check if this successor is in open set
        //if discovered a new node (not in open_set)
        if ((!open_set_ptrs.count(successor->id)) || (open_set_ptrs.size() == 0)){
            open_set.push(successor);
            open_set_ptrs.insert({successor->id, successor});
        }
        //else check if this path is better than previous path
        else {
            if (open_set_ptrs[successor->id]->g_cost > successor->g_cost){
                open_set_ptrs[successor->id] = successor;
            }
        }
    }
}

Eigen::MatrixXd Astar::getMotions() {
    // get available motions at each node
    Eigen::MatrixXd motion(26,4);
    // dx, dy, dz, cost
    motion << 1.0, 0.0, 0.0, 1.0,
              0.0, 1.0, 0.0, 1.0,
              -1.0, 0.0, 0.0, 1.0,
              0.0, -1.0, 0.0, 1.0,
              1.0, 1.0, 0.0, sqrt(2),
              -1.0, 1.0, 0.0, sqrt(2),
              -1.0, -1.0, 0.0, sqrt(2),
              1.0, -1.0, 0.0, sqrt(2),
              0.0, 0.0, 1.0, 1.0,
              0.0, 0.0, -1.0, 1.0,
              0.0, 1.0, 1.0, sqrt(2),
              0.0, -1.0, 1.0, sqrt(2),
              0.0, -1.0, -1.0, sqrt(2),
              0.0, 1.0, -1.0, sqrt(2),
              1.0, 0.0, 1.0, sqrt(2),
              -1.0, 0.0, 1.0, sqrt(2),
              -1.0, 0.0, -1.0, sqrt(2),
              1.0, 0.0, -1.0, sqrt(2),
              1.0, 1.0, 1.0, sqrt(3),
              -1.0, 1.0, 1.0, sqrt(3),
              -1.0, -1.0, 1.0, sqrt(3),
              1.0, -1.0, 1.0, sqrt(3),
              1.0, -1.0, -1.0, sqrt(3),
              -1.0, -1.0, -1.0, sqrt(3),
              -1.0, 1.0, -1.0, sqrt(3),
              1.0, 1.0, -1.0, sqrt(3);
    
    motion = motion*this->resolution;
    
    return motion;
}
