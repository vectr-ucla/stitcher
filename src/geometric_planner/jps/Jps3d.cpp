#include <iostream>

#include "Jps3d.h"

Jps::Jps(){
}

Jps::~Jps(){
    // ROS deletes stuff automatically
}

void Jps::Initialize(const double resolution, const double map_buffer){
    // get size of the map and bounds
    this->resolution = resolution;
    this->map_buffer = map_buffer;
    this->graph_lim = Eigen::MatrixXd::Zero(3,2);
    this->path = {};
    
    jn3d_ = std::make_shared<JPS3DNeib>();
    jn3d_->resolution = resolution;
    jn3d_->setResolution();
}

void Jps::plan(Eigen::Vector3d start, Eigen::Vector3d goal)
{
    // sort through best motions and develop path
    this->path.clear();

    this->start_node.x = start(0);
    this->start_node.y = start(1);
    this->start_node.z = start(2);
    this->start_node.dx = 0;
    this->start_node.dy = 0;
    this->start_node.dz = 0;
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

    char c;
    int nodes_searched = 0;

    while(open_set.size()>0){

        current = open_set.top();
        nodes_searched++;

        open_set.pop();
        open_set_ptrs.erase(current->id);
        closed_set.insert({current->id,current});

        // check if goal node is reached
        if (current->id == this->goal_node.id){
            // goal reached
            std::cout << "Jps Goal Reached!" << std::endl;
           // std::cout << "TOTAL NODES SEARCHED: " << nodes_searched << std::endl;

            this->goal_node.parent = current->parent;
            break;
        }

        // get successors of current node and add relevant ones to open set
        getSuccessors(current, open_set, open_set_ptrs, closed_set);

    }

    //goal not reached
    if (current->id != this->goal_node.id){
        std::cout << "No JPS Path found!" << std::endl;
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

void Jps::getSuccessors(Node* curr, 
                        std::priority_queue<Node*, std::vector<Node*>, Jps::CompareCost> &open_set, 
                        std::unordered_map<int, Node*> &open_set_ptrs, 
                        std::unordered_map<int, Node*> &closed_set){

    const int norm1 = (curr->dx != 0) + (curr->dy != 0) + (curr->dz != 0);
    int num_neib = jn3d_->nsz[norm1][0];
    int num_fneib = jn3d_->nsz[norm1][1];
    int dir_id = (curr->dx / this->resolution + 1) + 3*(curr->dy / this->resolution + 1) + 9*(curr->dz / this->resolution + 1);

    // for neighbor nodes
    for(int dev = 0; dev < num_neib+num_fneib; ++dev) {
        double new_x, new_y, new_z;
        double dx, dy, dz;

        // for non-forced neighbors
        if(dev < num_neib) {
          dx = jn3d_->ns[dir_id][0][dev];
          dy = jn3d_->ns[dir_id][1][dev];
          dz = jn3d_->ns[dir_id][2][dev];

          if(!jump(curr->x, curr->y, curr->z,
                  dx, dy, dz, new_x, new_y, new_z)) continue;
        }
        // check for possible forced neighbors
        else {
          double nx = curr->x + jn3d_->f1[dir_id][0][dev-num_neib];
          double ny = curr->y + jn3d_->f1[dir_id][1][dev-num_neib];
          double nz = curr->z + jn3d_->f1[dir_id][2][dev-num_neib];

          if(!verifyNode(nx,ny,nz)) {
              dx = jn3d_->f2[dir_id][0][dev-num_neib];
              dy = jn3d_->f2[dir_id][1][dev-num_neib];
              dz = jn3d_->f2[dir_id][2][dev-num_neib];

              if(!jump(curr->x, curr->y, curr->z,
                  dx, dy, dz, new_x, new_y, new_z)) continue;
          }
          else
              continue;
        }

        Node* succ = new Node();
        succ->x = new_x;
        succ->y = new_y;
        succ->z = new_z;
        succ->dx = dx;
        succ->dy = dy;
        succ->dz = dz;
        double move_cost = sqrt(pow(new_x - curr->x, 2.0) + pow(new_y - curr->y, 2.0) + pow(new_z - curr->z, 2.0));
        succ->g_cost = move_cost + curr->g_cost;
        succ->h_cost = calcHeuristic(*succ, this->goal_node);
        succ->parent = closed_set[curr->id];
        setNodeIndex(succ);

        if (!verifyNode(*succ)){
          continue;
        }

        if (closed_set.count(succ->id)){
          continue;
        }

        //check if this successor is in open set
        //if discovered a new node (not in open_set)
        if ((!open_set_ptrs.count(succ->id)) || (open_set_ptrs.size() == 0)){
            open_set.push(succ);
            open_set_ptrs.insert({succ->id, succ});
        }
        //else check if this path is better than previous path
        else {
            if (open_set_ptrs[succ->id]->g_cost > succ->g_cost){
                open_set_ptrs[succ->id] = succ;
            }
        }

    }    

}

bool Jps::jump(double x, double y, double z, double dx, double dy, double dz, double& new_x, double& new_y, double& new_z) {
  //Recursive function jump returns true if direction of next node has either
  //a forced neighbor or is the goal node, returns false if next node is off the map
  new_x = x + dx;
  new_y = y + dy;
  new_z = z + dz;

  
  if (!verifyNode(new_x, new_y, new_z)) {
    return false;
  }

  int new_id = coordToId(new_x, new_y, new_z);
  if (new_id == this->goal_node.id) {
    return true;
  }

  if (hasForced(new_x, new_y, new_z, dx, dy, dz)) {
    return true;
  }
  

  const int dir_id = (dx / this->resolution + 1) + 3*(dy / this->resolution + 1) + 9*(dz / this->resolution + 1);
  const int norm1 = (dx != 0) + (dy != 0) + (dz != 0);
  int num_neib = jn3d_->nsz[norm1][0];

  for( int k = 0; k < num_neib-1; ++k )
  {
    double new_new_x, new_new_y, new_new_z;
    if(jump(new_x,new_y,new_z,
          jn3d_->ns[dir_id][0][k], jn3d_->ns[dir_id][1][k], jn3d_->ns[dir_id][2][k],
        new_new_x, new_new_y, new_new_z)) return true;
  }

  // check for straight ahead neighbor, return when special condition is met
  return jump(new_x, new_y, new_z, dx, dy, dz, new_x, new_y, new_z);
}


inline bool Jps::hasForced(double x, double y, double z, double dx, double dy, double dz) {
  int norm1 = (dx != 0) + (dy != 0) + (dz != 0);
  int dir_id = (dx / this->resolution + 1) + 3*(dy / this->resolution + 1) + 9*(dz / this->resolution + 1);

  switch(norm1)
  {
    case 1:
      // 1-d move, check 8 neighbors
      for( int fn = 0; fn < 8; ++fn )
      {
        double nx = x + jn3d_->f1[dir_id][0][fn];
        double ny = y + jn3d_->f1[dir_id][1][fn];
        double nz = z + jn3d_->f1[dir_id][2][fn];
        if( !verifyNode(nx,ny,nz) )
          return true;
      }
      return false;
    case 2:
      // 2-d move, check 8 neighbors
      for( int fn = 0; fn < 8; ++fn )
      {
        double nx = x + jn3d_->f1[dir_id][0][fn];
        double ny = y + jn3d_->f1[dir_id][1][fn];
        double nz = z + jn3d_->f1[dir_id][2][fn];
        if(!verifyNode(nx,ny,nz))
          return true;
      }
      return false;
    case 3:
      // 3-d move, check 6 neighbors
      for( int fn = 0; fn < 6; ++fn )
      {
        double nx = x + jn3d_->f1[dir_id][0][fn];
        double ny = y + jn3d_->f1[dir_id][1][fn];
        double nz = z + jn3d_->f1[dir_id][2][fn];
        if( !verifyNode(nx,ny,nz) )
          return true;
      }
      return false;
    default:
      return false;
  }
}


JPS3DNeib::JPS3DNeib() {
  int dir_id = 0;
  for(int dz = -1; dz <= 1; ++dz) {
    for(int dy = -1; dy <= 1; ++dy) {
      for(int dx = -1; dx <= 1; ++dx) {

        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        for(int dev = 0; dev < nsz[norm1][0]; ++dev)
          Neib(dx,dy,dz,norm1,dev,
              ns[dir_id][0][dev], ns[dir_id][1][dev], ns[dir_id][2][dev]);
        for(int dev = 0; dev < nsz[norm1][1]; ++dev)
        {
          FNeib(dx,dy,dz,norm1,dev,
              f1[dir_id][0][dev],f1[dir_id][1][dev], f1[dir_id][2][dev],
              f2[dir_id][0][dev],f2[dir_id][1][dev], f2[dir_id][2][dev]);
        }
        dir_id ++;
      }
    }
  }
}

void JPS3DNeib::setResolution() {

    for (auto& r1 : this->ns) {
      for (auto& r2 : r1) {
        for (auto& element : r2) {
          element *= this->resolution;
          if (element < 0.1 && element > -0.1) {
            element = 0;
          }
        }
      }
    }

    for (auto& r1 : this->f1) {
      for (auto& r2 : r1) {
        for (auto& element : r2) {
          element *= this->resolution;
          if (element < 0.1 && element > -0.1) {
            element = 0;
          }
        }
      }
    }

    for (auto& r1 : this->f2) {
      for (auto& r2 : r1) {
        for (auto& element : r2) {
          element *= this->resolution;
          if (element < 0.1 && element > -0.1) {
            element = 0;
          }
        }
      }
    }
}

void JPS3DNeib::Neib(double dx, double dy, double dz, int norm1, int dev, double& tx, double& ty, double& tz)
{
  switch(norm1)
  {
    case 0:
      switch(dev)
      {
        case 0: tx=1; ty=0; tz=0; return;
        case 1: tx=-1; ty=0; tz=0; return;
        case 2: tx=0; ty=1; tz=0; return;
        case 3: tx=0; ty=-1; tz=0; return;
        case 4: tx=0; ty=0; tz=1; return;
        case 5: tx=0; ty=0; tz=-1; return;
        case 6: tx=1; ty=1; tz=0; return;
        case 7: tx=-1; ty=1; tz=0; return;
        case 8: tx=1; ty=-1; tz=0; return;
        case 9: tx=-1; ty=-1; tz=0; return;
        case 10: tx=1; ty=0; tz=1; return;
        case 11: tx=-1; ty=0; tz=1; return;
        case 12: tx=1; ty=0; tz=-1; return;
        case 13: tx=-1; ty=0; tz=-1; return;
        case 14: tx=0; ty=1; tz=1; return;
        case 15: tx=0; ty=-1; tz=1; return;
        case 16: tx=0; ty=1; tz=-1; return;
        case 17: tx=0; ty=-1; tz=-1; return;
        case 18: tx=1; ty=1; tz=1; return;
        case 19: tx=-1; ty=1; tz=1; return;
        case 20: tx=1; ty=-1; tz=1; return;
        case 21: tx=-1; ty=-1; tz=1; return;
        case 22: tx=1; ty=1; tz=-1; return;
        case 23: tx=-1; ty=1; tz=-1; return;
        case 24: tx=1; ty=-1; tz=-1; return;
        case 25: tx=-1; ty=-1; tz=-1; return;
      }
    case 1:
      tx = dx; ty = dy; tz = dz; return;
    case 2:
      switch(dev)
      {
        case 0:
          if(dz == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = 0; ty = 0; tz = dz; return;
          }
        case 1:
          if(dx == 0){
            tx = 0; ty = dy; tz = 0; return;
          }else{
            tx = dx; ty = 0; tz = 0; return;
          }
        case 2:
          tx = dx; ty = dy; tz = dz; return;
      }
    case 3:
      switch(dev)
      {
        case 0: tx = dx; ty =  0; tz =  0; return;
        case 1: tx =  0; ty = dy; tz =  0; return;
        case 2: tx =  0; ty =  0; tz = dz; return;
        case 3: tx = dx; ty = dy; tz =  0; return;
        case 4: tx = dx; ty =  0; tz = dz; return;
        case 5: tx =  0; ty = dy; tz = dz; return;
        case 6: tx = dx; ty = dy; tz = dz; return;
      }
  }
}

void JPS3DNeib::FNeib(double dx, double dy, double dz, int norm1, int dev,
  double& fx, double& fy, double& fz,
  double& nx, double& ny, double& nz)
{
  switch(norm1)
  {
    case 1:
      switch(dev)
      {
        case 0: fx= 0; fy= 1; fz = 0; break;
        case 1: fx= 0; fy=-1; fz = 0; break;
        case 2: fx= 1; fy= 0; fz = 0; break;
        case 3: fx= 1; fy= 1; fz = 0; break;
        case 4: fx= 1; fy=-1; fz = 0; break;
        case 5: fx=-1; fy= 0; fz = 0; break;
        case 6: fx=-1; fy= 1; fz = 0; break;
        case 7: fx=-1; fy=-1; fz = 0; break;
      }
      nx = fx; ny = fy; nz = dz;
      // switch order if different direction
      if(dx != 0){
        fz = fx; fx = 0;
        nz = fz; nx = dx;
      }if(dy != 0){
        fz = fy; fy = 0;
        nz = fz; ny = dy;
      }
      return;
    case 2:
      if(dx == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = 0; ny = dy; nz = -dz;
            return;
          case 1:
            fx = 0; fy = -dy; fz = 0;
            nx = 0; ny = -dy; nz = dz;
            return;
          case 2:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = dz;
            return;
          case 3:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = dz;
            return;
          case 4:
            fx = 1; fy = 0; fz = -dz;
            nx = 1; ny = dy; nz = -dz;
            return;
          case 5:
            fx = 1; fy = -dy; fz = 0;
            nx = 1; ny = -dy; nz = dz;
            return;
          case 6:
            fx = -1; fy = 0; fz = -dz;
            nx = -1; ny = dy; nz = -dz;
            return;
          case 7:
            fx = -1; fy = -dy; fz = 0;
            nx = -1; ny = -dy; nz = dz;
            return;
          // Extras
          case 8:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = dy; nz = 0;
            return;
          case 9:
            fx = 1; fy = 0; fz = 0;
            nx = 1; ny = 0; nz = dz;
            return;
          case 10:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = dy; nz = 0;
            return;
          case 11:
            fx = -1; fy = 0; fz = 0;
            nx = -1; ny = 0; nz = dz;
            return;
        }
      }else if(dy == 0){
        switch(dev)
        {
          case 0:
            fx = 0; fy = 0; fz = -dz;
            nx = dx; ny = 0; nz = -dz;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = 0; nz = dz;
            return;
          case 2:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = dz;
            return;
          case 3:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1;nz = dz;
            return;
          case 4:
            fx = 0; fy = 1; fz = -dz;
            nx = dx; ny = 1; nz = -dz;
            return;
          case 5:
            fx = -dx; fy = 1; fz = 0;
            nx = -dx; ny = 1; nz = dz;
            return;
          case 6:
            fx = 0; fy = -1; fz = -dz;
            nx = dx; ny = -1; nz = -dz;
            return;
          case 7:
            fx = -dx; fy = -1; fz = 0;
            nx = -dx; ny = -1; nz = dz;
            return;
          // Extras
          case 8:
            fx = 0; fy = 1; fz = 0;
            nx = dx; ny = 1; nz = 0;
            return;
          case 9:
            fx = 0; fy = 1; fz = 0;
            nx = 0; ny = 1; nz = dz;
            return;
          case 10:
            fx = 0; fy = -1; fz = 0;
            nx = dx; ny = -1; nz = 0;
            return;
          case 11:
            fx = 0; fy = -1; fz = 0;
            nx = 0; ny = -1; nz = dz;
            return;
        }
      }else{// dz==0
        switch(dev)
        {
          case 0:
            fx = 0; fy = -dy; fz = 0;
            nx = dx; ny = -dy; nz = 0;
            return;
          case 1:
            fx = -dx; fy = 0; fz = 0;
            nx = -dx; ny = dy; nz = 0;
            return;
          case 2:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = dy; nz = 1;
            return;
          case 3:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = dy; nz = -1;
            return;
          case 4:
            fx = 0; fy = -dy; fz = 1;
            nx = dx; ny = -dy; nz = 1;
            return;
          case 5:
            fx = -dx; fy = 0; fz = 1;
            nx = -dx; ny = dy; nz = 1;
            return;
          case 6:
            fx = 0; fy = -dy; fz = -1;
            nx = dx; ny = -dy; nz = -1;
            return;
          case 7:
            fx = -dx; fy = 0; fz = -1;
            nx = -dx; ny = dy; nz = -1;
            return;
          // Extras
          case 8:
            fx =  0; fy = 0; fz = 1;
            nx = dx; ny = 0; nz = 1;
            return;
          case 9:
            fx = 0; fy = 0; fz = 1;
            nx = 0; ny = dy; nz = 1;
            return;
          case 10:
            fx =  0; fy = 0; fz = -1;
            nx = dx; ny = 0; nz = -1;
            return;
          case 11:
            fx = 0; fy = 0; fz = -1;
            nx = 0; ny = dy; nz = -1;
            return;
        }
      }
    case 3:
      switch(dev)
      {
        case 0:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = dz;
          return;
        case 1:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = dz;
          return;
        case 2:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = dy; nz = -dz;
          return;
        // Need to check up to here for forced!
        case 3:
          fx = 0; fy = -dy; fz = -dz;
          nx = dx; ny = -dy; nz = -dz;
          return;
        case 4:
          fx = -dx; fy = 0; fz = -dz;
          nx = -dx; ny = dy; nz = -dz;
          return;
        case 5:
          fx = -dx; fy = -dy; fz = 0;
          nx = -dx; ny = -dy; nz = dz;
          return;
        // Extras
        case 6:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = 0; nz = dz;
          return;
        case 7:
          fx = -dx; fy = 0; fz = 0;
          nx = -dx; ny = dy; nz = 0;
          return;
        case 8:
          fx = 0; fy = -dy; fz = 0;
          nx = 0; ny = -dy; nz = dz;
          return;
        case 9:
          fx = 0; fy = -dy; fz = 0;
          nx = dx; ny = -dy; nz = 0;
          return;
        case 10:
          fx = 0; fy = 0; fz = -dz;
          nx = 0; ny = dy; nz = -dz;
          return;
        case 11:
          fx = 0; fy = 0; fz = -dz;
          nx = dx; ny = 0; nz = -dz;
          return;
      }
  }
}