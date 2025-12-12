#include "ROS_Goal_Click_Planning.h"
#include "signal.h"

// Prototypes
void controlC(int sig);

int main(int argc, char **argv) 
{
	//Initializes ROS, and sets up a node
	ros::init(argc,argv,"stitcher_planner");
	ros::NodeHandle nh("~");

	/* Setup the CTRL-C trap */
	signal(SIGINT, controlC);
  sleep(0.5);

  Motion_Primitives::RosNode::LocalPlanner stitcher_planner(nh);
  stitcher_planner.Start();
  ros::AsyncSpinner spinner(7);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}

//## Custom Control-C handler
void controlC(int sig)
{
  Motion_Primitives::RosNode::LocalPlanner::Abort();
}