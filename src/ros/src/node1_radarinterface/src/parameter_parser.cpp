#include "parameter_parser.h"

namespace node1_radarinterface {

PLUGINLIB_EXPORT_CLASS(node1_radarinterface::parameter_parser, nodelet::Nodelet);

parameter_parser::parameter_parser() {}

void parameter_parser::onInit() {}

void parameter_parser::params_parser(node1_radarinterface::mmWaveCLI &srv, ros::NodeHandle &nh) {

//   ROS_ERROR("%s",srv.request.comm.c_str());
//   ROS_ERROR("%s",srv.response.resp.c_str());
  std::vector <std::string> v;
  std::string s = srv.request.comm.c_str(); 
  std::istringstream ss(s);
  std::string token;
  std::string req;
  int i = 0;
  while (std::getline(ss, token, ' ')) {
    v.push_back(token);
    if (i > 0) {
      if (!req.compare("profileCfg")) {
        switch (i) {
          case 2:
            nh.setParam("/node1_radarinterface/startFreq", std::stof(token)); break;
          case 3:
            nh.setParam("/node1_radarinterface/idleTime", std::stof(token)); break;
          case 4:
            nh.setParam("/node1_radarinterface/adcStartTime", std::stof(token)); break;
          case 5:
            nh.setParam("/node1_radarinterface/rampEndTime", std::stof(token)); break;
          case 8:
            nh.setParam("/node1_radarinterface/freqSlopeConst", std::stof(token)); break;
          case 10:
            nh.setParam("/node1_radarinterface/numAdcSamples", std::stoi(token)); break;
          case 11:
            nh.setParam("/node1_radarinterface/digOutSampleRate", std::stof(token)); break;
          case 14:
            nh.setParam("/node1_radarinterface/rxGain", std::stof(token)); break;
        }
      } else if (!req.compare("frameCfg")) {
        switch (i) {
          case 1:
            nh.setParam("/node1_radarinterface/chirpStartIdx", std::stoi(token)); break;
          case 2:
            nh.setParam("/node1_radarinterface/chirpEndIdx", std::stoi(token)); break;
          case 3:
            nh.setParam("/node1_radarinterface/numLoops", std::stoi(token)); break;
          case 4:
            nh.setParam("/node1_radarinterface/numFrames", std::stoi(token)); break;
          case 5:
            nh.setParam("/node1_radarinterface/framePeriodicity", std::stof(token)); break;
        }
      }
    } else req = token;
    i++;
  }
}

void parameter_parser::cal_params(ros::NodeHandle &nh) {
  float c0 = 299792458;
  int chirpStartIdx;
  int chirpEndIdx;
  int numLoops;
  float framePeriodicity;
  float startFreq;
  float idleTime;
  float adcStartTime;
  float rampEndTime;
  float digOutSampleRate;
  float freqSlopeConst;
  float numAdcSamples;

  nh.getParam("/node1_radarinterface/startFreq", startFreq);
  nh.getParam("/node1_radarinterface/idleTime", idleTime);
  nh.getParam("/node1_radarinterface/adcStartTime", adcStartTime);
  nh.getParam("/node1_radarinterface/rampEndTime", rampEndTime);
  nh.getParam("/node1_radarinterface/digOutSampleRate", digOutSampleRate);
  nh.getParam("/node1_radarinterface/freqSlopeConst", freqSlopeConst);
  nh.getParam("/node1_radarinterface/numAdcSamples", numAdcSamples);

  nh.getParam("/node1_radarinterface/chirpStartIdx", chirpStartIdx);
  nh.getParam("/node1_radarinterface/chirpEndIdx", chirpEndIdx);
  nh.getParam("/node1_radarinterface/numLoops", numLoops);
  nh.getParam("/node1_radarinterface/framePeriodicity", framePeriodicity);

  int ntx = chirpEndIdx - chirpStartIdx + 1;
  int nd = numLoops;
  int nr = numAdcSamples;
  float tfr = framePeriodicity * 1e-3;
  float fs = digOutSampleRate * 1e3;
  float kf = freqSlopeConst * 1e12;
  float adc_duration = nr / fs;
  float BW = adc_duration * kf;
  float PRI = (idleTime + rampEndTime) * 1e-6;
  float fc = startFreq * 1e9 + kf * (adcStartTime * 1e-6 + adc_duration / 2); 
  float fc_chirp = startFreq * 1e9 + BW / 2; 

  float vrange = c0 / (2 * BW);
  float max_range = nr * vrange;
  float max_vel = c0 / (2 * fc * PRI) / ntx;
  float vvel = max_vel / nd;

  nh.setParam("/node1_radarinterface/num_TX", ntx);
  nh.setParam("/node1_radarinterface/f_s", fs);
  nh.setParam("/node1_radarinterface/f_c", fc);
  nh.setParam("/node1_radarinterface/fc_chirp", fc_chirp);
  nh.setParam("/node1_radarinterface/BW", BW);
  nh.setParam("/node1_radarinterface/PRI", PRI);
  nh.setParam("/node1_radarinterface/t_fr", tfr);
  nh.setParam("/node1_radarinterface/max_range", max_range);
  nh.setParam("/node1_radarinterface/range_resolution", vrange);
  nh.setParam("/node1_radarinterface/max_doppler_vel", max_vel);
  nh.setParam("/node1_radarinterface/doppler_vel_resolution", vvel);
}

}