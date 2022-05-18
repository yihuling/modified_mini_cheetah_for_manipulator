#ifndef BACKFLIP_DATA_READER_H
#define BACKFLIP_DATA_READER_H
#include <cppTypes.h>
#include <FSM_States/FSM_State.h>
#include <FSM_States/ControlFSMData.h>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <string>

enum plan_offsets {
  q0_offset = 0,     // x, z, yaw, front hip, front knee, rear hip, rear knee
  qd0_offset = 7,    // x, z, yaw, front hip, front knee, rear hip, rear knee
  tau_offset = 24,//14,   // front hip, front knee, rear hip, rear knee
  force_offset = 18  // front x, front z, rear x, rear z
};

typedef Eigen::Matrix<float, 7, 1> Vector7f;

class DataReader {
 public:

  std::string motion_type_name[5]; //front,left,right,rear,turn
  int _count_path;
  std::vector<float> best_energy;
  std::string motion_subtype_name; //flip,jump,two_step_jump
  std::string motion_steptype_name; //step_1,step_2--->
  std::string motion_index_name; //i_x---

  std::string motion_index_two_step_name; //i_x---

  static const int _opt_plan_cols = 24;
  DataReader(MIT_UserParameters* parameters);

  void load_control_plan(const char *filename);
  void unload_control_plan();
  float *get_plan_at_time(int timestep);
  //data type=[pos,orientation,A,B,C,D]
  void load_data_index_from_yaml(const char *yamlfilename);
  int plan_timesteps = -1;
  bool areEqual(const Vec3<float>& p1, const Vec3<float>& p2,float_t EPSILON);
  MIT_UserParameters* _parameters = nullptr;
  bool estop_for_motions =false;

 private:

//  RobotType _type;
  float *plan_buffer;
  bool plan_loaded = false;
  std::string path_name_file;//will be delete after use kin in this repo
  Vec12<float> optimal_data;// TODO? lz

};

#endif  // BACKFLIP_DATA_READER_H
