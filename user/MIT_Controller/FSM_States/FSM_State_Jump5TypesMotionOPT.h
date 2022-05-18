#ifndef FSM_State_FrontJumpOPT_OPT_H
#define FSM_State_FrontJumpOPT_OPT_H

#include "FSM_State.h"
#include <Controllers/JumpMotion/DataReader.hpp>
#include <Controllers/JumpMotion/Jump5TypesMotionOPT.hpp>

/**
 *
 */
template <typename T>
class FSM_State_Jump5TypesMotionOPT : public FSM_State<T> {
 public:
  FSM_State_Jump5TypesMotionOPT(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

 private:
  // Keep track of the control iterations
  int iter = 0;
  int _motion_start_iter = 0;

  static constexpr int Preparation = 0;
  static constexpr int Flip = 1;
  static constexpr int Landing = 2;

  unsigned long long _state_iter;
  int _flag = Preparation;
  Vec3<T> trajOPTAll[2000];
    Vec3<T> trajOPTAllReal[2000];
  int _jump_count;
  bool _jump_over_flag;
  bool _stand_up_in_landing_yaw= true;
  // JPos
  Vec3<T> initial_jpos[4];
  Vec3<T> zero_vec3;
  Vec3<T> f_ff;
  
  void _SetJPosInterPts(
      const size_t & curr_iter, size_t max_iter, int leg, 
      const Vec3<T> & ini, const Vec3<T> & fin);

  DataReader* _data_reader;
  bool _b_running = true;
  bool _b_first_visit = true;
  int _count = 0;
   int _count_yaw_stand=0;
  int _waiting_count = 6;

  float _curr_time = 0;
    bool enter_once;
    bool enter_once_withdraw;
  Jump5TypesMotionOPT<T>* front_jump_ctrl_;
    Vec3<T> inip,finp,ff_force;
    Vec3<T> initial_Ppos[4];
    float real_v_world[3];
    float real_p_body_to_world[3];
    float real_omega_b2w[3];
    float _real_p2w_inter_with_imu[3];
    Mat3 <T> kpMat,kdMat;
  void SetTestParameter(const std::string& test_file);
  Vec3<T> InverseKinematics(Vec3<T> pos);
  bool _Initialization();
  void ComputeCommand();
  void _SafeCommand();

};

#endif  // FSM_State_FrontJumpOPT_H
