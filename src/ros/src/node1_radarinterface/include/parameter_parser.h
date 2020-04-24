#ifndef _PARAM_PARSER_CLASS_
#define _PARAM_PARSER_CLASS_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "node1_radarinterface/mmWaveCLI.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>

namespace node1_radarinterface {

class parameter_parser : public nodelet::Nodelet{

  public:
  	
  	parameter_parser();
  	void params_parser(node1_radarinterface::mmWaveCLI &srv, ros::NodeHandle &n);
	void cal_params(ros::NodeHandle &nh);

  private:
    
    virtual void onInit();
    
    node1_radarinterface::mmWaveCLI srv;

};
}
#endif
