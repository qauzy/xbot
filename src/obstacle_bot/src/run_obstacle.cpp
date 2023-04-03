#include "obstacle.h"
int main(int argc, char **argv)
{
    // Initiate new ROS node named }stopper}
    ros::init(argc, argv, "Obstacle");
    // Create new stopper object
    Obstacle obstacle;
    // Start the movement
    obstacle.startMoving();
    return 0;
}

