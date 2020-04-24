#include "ros/ros.h"
#include "nodelet/loader.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mmWave_Manager");
  
  nodelet::Loader manager(true);
  
  nodelet::M_string remap(ros::names::getRemappings());
  
  nodelet::V_string nargv;
  
  manager.load("mmWaveCommSrv", "node1_radarinterface/mmWaveCommSrv", remap, nargv);
  
  manager.load("mmWaveDataHdl", "node1_radarinterface/mmWaveDataHdl", remap, nargv);
  
  ros::spin();
  
  return 0;
  }
