#include "approach_grab.h"



int main(int argc, char **argv)
{
   ros::init(argc, argv, "approach_grab_node");
   approach_grab mygrab;
   mygrab.spin();
   return 0;
}