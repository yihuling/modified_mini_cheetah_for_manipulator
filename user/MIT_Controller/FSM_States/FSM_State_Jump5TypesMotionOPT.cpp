/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */
//#include "Math/Interpolation.h"
#include "FSM_State_Jump5TypesMotionOPT.h"
#include <Utilities/Utilities_print.h>
#include <Configuration.h>
#include "Math/Interpolation.h"
#include <fstream>
#define PI (3.141592654f)
//#define DRAW_DEBUG_PATH
#define SAVE_DEBUG_DATA

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Jump5TypesMotionOPT<T>::FSM_State_Jump5TypesMotionOPT(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  zero_vec3.setZero();
  f_ff << 0.f, 0.f, -25.f;

    _data_reader = new DataReader(_controlFSMData->userParameters);

    front_jump_ctrl_ = new Jump5TypesMotionOPT<T>(_data_reader, this->_data->controlParameters->controller_dt);

    front_jump_ctrl_->SetParameter();
}


template <typename T>
void FSM_State_Jump5TypesMotionOPT<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
  _state_iter = 0;
  _count = 0;
  _curr_time = 0;
    _jump_count=0;
  _motion_start_iter = 0;
    _count_yaw_stand=0;
  _b_first_visit = true;
  _jump_over_flag= false;
    _stand_up_in_landing_yaw=true;
  _data_reader->load_data_index_from_yaml(THIS_COM "config/jumpmotiondata/jump_motion_library_index.yaml");
//  _data_reader->load_control_plan(THIS_COM "config/jump_side_opt.dat",36);

  // initial configuration, position
  for(size_t i(0); i < 4; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
  }
  front_jump_ctrl_->SetParameter();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Jump5TypesMotionOPT<T>::run() {

// Command Computation
  if (_b_running) {
    if (!_Initialization()) {
      ComputeCommand();
    }
  } else {
    _SafeCommand();
  }

  ++_count;

  _curr_time += this->_data->controlParameters->controller_dt;

}


template <typename T>
bool FSM_State_Jump5TypesMotionOPT<T>::_Initialization() { // do away with this?
  static bool test_initialized(false);
  if (!test_initialized) {
    test_initialized = true;
    printf("[Cheetah Test] Test initialization is done\n");
  }
  if (_count < _waiting_count) {
    for (int leg = 0; leg < 4; ++leg) {
      this->_data->_legController->commands[leg].qDes = initial_jpos[leg];
      for (int jidx = 0; jidx < 3; ++jidx) {
        this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
        this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
        this->_data->_legController->commands[leg].kpJoint(jidx,jidx) = 80.;
        this->_data->_legController->commands[leg].kdJoint(jidx,jidx) = 2.;
      }
    }
    return true;
  }
  
  return false;
}

template <typename T>
void FSM_State_Jump5TypesMotionOPT<T>::ComputeCommand() {
    auto& seResult =this->_data->_stateEstimator->getResult();
    real_v_world[0]=seResult.vWorld[0];
    real_v_world[1]=seResult.vWorld[1];
    real_v_world[2]=seResult.vWorld[2];
    real_p_body_to_world[0]=seResult.position[0];
    real_p_body_to_world[1]=seResult.position[1];
    real_p_body_to_world[2]=seResult.position[2];
    real_omega_b2w[0]=seResult.rpy[1];
    real_omega_b2w[1]=seResult.omegaWorld[1];
    float l1 = this->_data->_quadruped->_hipLinkLength;
    float l2 = this->_data->_quadruped->_kneeLinkLength;
    if (_count < 500) { //准备起跳


        //求站起来时，hip与knee角度
        float h  = 0.15;
        float theta1 = -acosf((l1*l1+h*h-l2*l2)/(2*l1*h));
        float theta2 = PI - acosf((l1*l1+l2*l2-h*h)/(2*l1*l2));

        finp << 0, theta1,theta2;
        ff_force << 0, 0, -200;
        ff_force = ff_force * 0.0;
        for (int leg = 0; leg < 2; leg++) //前腿
        {
            _SetJPosInterPts(_count, 200, leg, initial_jpos[leg], finp);
            this->_data->_legController->commands[leg].forceFeedForward = ff_force;
        }

        finp << 0, theta1,theta2;
        ff_force << -300, 0, -250;
        ff_force = ff_force * 0;
        for (int leg = 2; leg < 4; leg++) //后腿
        {
            _SetJPosInterPts(_count, 200, leg, initial_jpos[leg], finp);
            this->_data->_legController->commands[leg].forceFeedForward = ff_force;
        }


    }else {

        if (!_data_reader->estop_for_motions &&!_jump_over_flag) {
            if (_b_first_visit) {
                front_jump_ctrl_->FirstVisit(_curr_time);
                _b_first_visit = false;
            }

            front_jump_ctrl_->OneStep(_curr_time, false, this->_data->_legController->commands);

            if (front_jump_ctrl_->EndOfPhase(this->_data->_legController->datas)) {
                front_jump_ctrl_->LastVisit();
                std::cout<<"Jump motion Over----"<<std::endl;


            } else {
#ifdef SAVE_DEBUG_DATA
                static std::ofstream log_jump("./log_jump.txt");//

                log_jump << _curr_time << " ";//1
                log_jump << this->_data->_legController->datas[0].q[0] << " ";//2
                log_jump << this->_data->_legController->datas[0].q[1] << " ";//3
                log_jump << this->_data->_legController->datas[0].q[2] << " ";//4

                log_jump << this->_data->_legController->datas[0].qd[0] << " ";//5
                log_jump << this->_data->_legController->datas[0].qd[1] << " ";//6
                log_jump << this->_data->_legController->datas[0].qd[2] << " ";//7

                log_jump << this->_data->_legController->datas[0].tauEstimate[0] << " ";//8
                log_jump << this->_data->_legController->datas[0].tauEstimate[1] << " ";//9
                log_jump << this->_data->_legController->datas[0].tauEstimate[2] << " ";//10
                log_jump << this->_data->_legController->datas[0].p[0] << " ";//11
                log_jump << this->_data->_legController->datas[0].p[1] << " ";//12
                log_jump << this->_data->_legController->datas[0].p[2] << " ";//13

                log_jump << this->_data->_legController->datas[3].q[0] << " ";//14
                log_jump << this->_data->_legController->datas[3].q[1] << " ";//15
                log_jump << this->_data->_legController->datas[3].q[2] << " ";//16

                log_jump << this->_data->_legController->datas[3].qd[0] << " ";//17
                log_jump << this->_data->_legController->datas[3].qd[1] << " ";//18
                log_jump << this->_data->_legController->datas[3].qd[2] << " ";//19

                log_jump << this->_data->_legController->datas[3].tauEstimate[0] << " ";//20
                log_jump << this->_data->_legController->datas[3].tauEstimate[1] << " ";//21
                log_jump << this->_data->_legController->datas[3].tauEstimate[2] << " ";//22

                log_jump << this->_data->_legController->datas[3].p[0] << " ";//23
                log_jump << this->_data->_legController->datas[3].p[1] << " ";//24
                log_jump << this->_data->_legController->datas[3].p[2] << " ";//25

                log_jump << this->_data->_legController->datas[1].q[0] << " ";//26
                log_jump << this->_data->_legController->datas[1].q[1] << " ";//27
                log_jump << this->_data->_legController->datas[1].q[2] << " ";//28

                log_jump << this->_data->_legController->datas[1].qd[0] << " ";//29
                log_jump << this->_data->_legController->datas[1].qd[1] << " ";//30
                log_jump << this->_data->_legController->datas[1].qd[2] << " ";//31

                log_jump << this->_data->_legController->datas[1].tauEstimate[0] << " ";//32
                log_jump << this->_data->_legController->datas[1].tauEstimate[1] << " ";//33
                log_jump << this->_data->_legController->datas[1].tauEstimate[2] << " ";//34

                log_jump << this->_data->_legController->datas[1].p[0] << " ";//35
                log_jump << this->_data->_legController->datas[1].p[1] << " ";//36
                log_jump << this->_data->_legController->datas[1].p[2] << " ";//37

                log_jump << this->_data->_legController->datas[2].q[0] << " ";//38
                log_jump << this->_data->_legController->datas[2].q[1] << " ";//39
                log_jump << this->_data->_legController->datas[2].q[2] << " ";//40

                log_jump << this->_data->_legController->datas[2].qd[0] << " ";//41
                log_jump << this->_data->_legController->datas[2].qd[1] << " ";//42
                log_jump << this->_data->_legController->datas[2].qd[2] << " ";//43

                log_jump << this->_data->_legController->datas[2].tauEstimate[0] << " ";//44
                log_jump << this->_data->_legController->datas[2].tauEstimate[1] << " ";//45
                log_jump << this->_data->_legController->datas[2].tauEstimate[2] << " ";//46

                log_jump << this->_data->_legController->datas[2].p[0] << " ";//47
                log_jump << this->_data->_legController->datas[2].p[1] << " ";//48
                log_jump << this->_data->_legController->datas[2].p[2] << " ";//49

                log_jump << this->_data->_legController->commands[0].qDes[0] << " ";//50
                log_jump << this->_data->_legController->commands[0].qDes[1] << " ";//51
                log_jump << this->_data->_legController->commands[0].qDes[2] << " ";//52

                log_jump << this->_data->_legController->commands[3].qDes[0] << " ";//53
                log_jump << this->_data->_legController->commands[3].qDes[1] << " ";//54
                log_jump << this->_data->_legController->commands[3].qDes[2] << " ";//55
                log_jump << seResult.position[0] << " ";//56
                log_jump << seResult.position[1] << " ";//57
                log_jump << seResult.position[2] << " ";//58
                log_jump << seResult.rpy[0] << " ";//59
                log_jump << seResult.rpy[1] << " ";//60
                log_jump << seResult.rpy[2] << " ";//61
                log_jump << seResult.omegaWorld[0] << " ";//62
                log_jump << seResult.omegaWorld[1] << " ";//63
                log_jump << seResult.omegaWorld[2] << " ";//64
                log_jump << seResult.vWorld[0] << " ";//65
                log_jump << seResult.vWorld[1] << " ";//66
                log_jump << seResult.vWorld[2] << " ";//67
                log_jump << this->_data->_legController->commands[0].pDes[0] << " "
                         << this->_data->_legController->commands[0].pDes[1] << " "
                         << this->_data->_legController->commands[0].pDes[2] << " ";//68-69-70
                log_jump << this->_data->_legController->commands[3].pDes[0] << " "
                         << this->_data->_legController->commands[3].pDes[1] << " "
                         << this->_data->_legController->commands[3].pDes[2] << " ";//71-72-73
                log_jump << this->_data->_legController->commands[0].vDes[0] << " "
                         << this->_data->_legController->commands[0].vDes[1] << " "
                         << this->_data->_legController->commands[0].vDes[2] << " ";//74-75-76
                log_jump << this->_data->_legController->commands[3].vDes[0] << " "
                         << this->_data->_legController->commands[3].vDes[1] << " "
                         << this->_data->_legController->commands[3].vDes[2] << " ";//77-78-79
                log_jump << this->_data->_legController->datas[0].v[0] << " "
                         << this->_data->_legController->datas[0].v[1]
                         << " " << this->_data->_legController->datas[0].v[2] << " ";//80-81-82
                log_jump << this->_data->_legController->datas[3].v[0] << " "
                         << this->_data->_legController->datas[3].v[1]
                         << " " << this->_data->_legController->datas[3].v[2] << " ";//83-84-85
                log_jump << this->_data->_legController->commands[1].qDes[0] << " ";//86
                log_jump << this->_data->_legController->commands[1].qDes[1] << " ";//87
                log_jump << this->_data->_legController->commands[1].qDes[2] << " ";//88

                log_jump << this->_data->_legController->commands[2].qDes[0] << " ";//89
                log_jump << this->_data->_legController->commands[2].qDes[1] << " ";//90
                log_jump << this->_data->_legController->commands[2].qDes[2] << " ";//91
                log_jump << this->_data->_legController->commands[0].qdDes[0] << " ";//92
                log_jump << this->_data->_legController->commands[0].qdDes[1] << " ";//93
                log_jump << this->_data->_legController->commands[0].qdDes[2] << " ";//94

                log_jump << this->_data->_legController->commands[1].qdDes[0] << " ";//95
                log_jump << this->_data->_legController->commands[1].qdDes[1] << " ";//96
                log_jump << this->_data->_legController->commands[1].qdDes[2] << " ";//97
                log_jump << this->_data->_legController->commands[2].qdDes[0] << " ";//98
                log_jump << this->_data->_legController->commands[2].qdDes[1] << " ";//99
                log_jump << this->_data->_legController->commands[2].qdDes[2] << " ";//100

                log_jump << this->_data->_legController->commands[3].qdDes[0] << " ";//101
                log_jump << this->_data->_legController->commands[3].qdDes[1] << " ";//102
                log_jump << this->_data->_legController->commands[3].qdDes[2] << " ";//103
                log_jump << std::endl;
#endif
            }
            _jump_count += 1;
            if (_jump_count > front_jump_ctrl_->get_plan_num()) {
                _jump_count = front_jump_ctrl_->get_plan_num() - 1;
                _jump_over_flag = true;
            }
        } else
        {
//            std::cout<<"Not safe motion,check your param or stand in normal"<<std::endl;
        }
    }
//    std::cout<<"_jump_over_flag: "<<_jump_over_flag<<" "<<_stand_up_in_landing_yaw<<"  "<<_count_yaw_stand<<std::endl;
    if (_jump_over_flag && _stand_up_in_landing_yaw)
    {
        ++_count_yaw_stand;
        if (_count_yaw_stand < 1200) { //准备起跳
            for(size_t i(0); i < 4; ++i) {
                initial_jpos[i] = this->_data->_legController->datas[i].q;
            }

            //求站起来时，hip与knee角度
            float h  = 0.15;
            float theta1 = -acosf((l1*l1+h*h-l2*l2)/(2*l1*h));
            float theta2 = PI - acosf((l1*l1+l2*l2-h*h)/(2*l1*l2));

            finp << 0, theta1,theta2;
            for (int leg = 0; leg < 4; leg++) //前腿
            {
                _SetJPosInterPts(_count_yaw_stand, 1200, leg, initial_jpos[leg], finp);

            }


        }else
        {
            for(size_t i(0); i < 4; ++i) {
                initial_jpos[i] = this->_data->_legController->datas[i].q;
            }

            //求站起来时，hip与knee角度
            float h  = 0.29;
            float theta1 = -acosf((l1*l1+h*h-l2*l2)/(2*l1*h));
            float theta2 = PI - acosf((l1*l1+l2*l2-h*h)/(2*l1*l2));

            finp << 0, theta1,theta2;

            for (int leg = 0; leg < 4; leg++) //前腿
            {
                _SetJPosInterPts(_count_yaw_stand, 5200, leg, initial_jpos[leg], finp);

            }

        }
    }
//    std::cout<<"joint "<<this->_data->_legController->commands[0].qDes[1] <<" "<<this->_data->_legController->commands[0].qDes[2]<<" "<<this->_data->_legController->commands[3].qDes[1]
//    <<" "<<this->_data->_legController->commands[3].qDes[2]<< std::endl;
#ifdef DRAW_DEBUG_PATH
    if (_jump_count<2000) { //the max path data
        auto *FirstSphere = this->_data->visualizationData->addSphere();
        if (FirstSphere) {
            FirstSphere->position = trajOPTAll[0];
            FirstSphere->radius = 0.02;
            FirstSphere->color = {1.0, 0.2, 0.2, 0.5};
        }
        auto *trajectoryrealDebug = this->_data->visualizationData->addPath();
        if (trajectoryrealDebug) {
            trajectoryrealDebug->num_points = _jump_count;
            trajectoryrealDebug->color = {1.0, 0.2, 0.2, 0.5};
            if (_jump_over_flag) {
                trajectoryrealDebug->position[_jump_count] = trajOPTAllReal[_jump_count];
            } else {
                for (int i = 0; i < int(trajectoryrealDebug->num_points); i++) {
                    trajectoryrealDebug->position[i] = trajOPTAllReal[i];
                }
            }
        }
        auto *trajectoryDebug = this->_data->visualizationData->addPath();
        if (trajectoryDebug) {
            trajectoryDebug->num_points = _jump_count;
            trajectoryDebug->color = {0.2, 1, 0.2, 0.5};
            if (_jump_over_flag) {
                trajectoryDebug->position[_jump_count] = trajOPTAll[_jump_count];
            } else {
                for (int i = 0; i < int(trajectoryDebug->num_points); i++) {
                    trajectoryDebug->position[i] = trajOPTAll[i];
                }
            }
        }
        auto *FinalSphere = this->_data->visualizationData->addSphere();
        if (FinalSphere) {
            FinalSphere->position = trajOPTAll[front_jump_ctrl_->curr_iter_for_plot];
            FinalSphere->radius = 0.02;
            FinalSphere->color = {1.0, 0.2, 0.2, 0.5};
        }
    }
#endif

}

template <typename T>
void FSM_State_Jump5TypesMotionOPT<T>::_SafeCommand() {
  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
      this->_data->_legController->commands[leg].tauFeedForward[jidx] = 0.;
      this->_data->_legController->commands[leg].qDes[jidx] = this->_data->_legController->datas[leg].q[jidx];
      this->_data->_legController->commands[leg].qdDes[jidx] = 0.;
    }
  }
}
template <typename T>
Vec3<T> FSM_State_Jump5TypesMotionOPT<T>::InverseKinematics(Vec3<T> pos)
{
    Vec3<T> ang;
    float hipx=pos(0);
//    double hipy=pos(1);
    float hipz=pos(2);
    float L1=0.209;
    float L2=0.195;
//    float PI=3.1415926;
    ang(0)=0;
    float L12=sqrt((hipz)*(hipz)+hipx*hipx);
    if(L12>0.38) L12=0.38;
    else if(L12<0.06) L12=0.06;
//printf("IK:L12:%.2f\n",L12);
    float fai=acos((L1*L1+L12*L12-L2*L2)/2.0/L1/L12);
//    if(isnan(fai))
//    { printf("IK:fai is nan :%.2f\t%.2f\n",L1*L1+L12*L12-L2*L2,(L1*L1+L12*L12-L2*L2)/2.0/L1/L12);}
//    printf("IK:fai:%.2f\n",fai);
    ang(1)=atan2(hipx,-hipz)-fai;
    ang(2)=PI-acos((L1*L1+L2*L2-L12*L12)/2.0/L1/L2);
    return ang;
}

template <typename T>
void FSM_State_Jump5TypesMotionOPT<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg, 
    const Vec3<T> & ini, const Vec3<T> & fin){

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    Vec3<T> inter_pos = a * ini + b * fin;

    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3);

    //if(curr_iter == 0){ 
      //printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n", 
        //_flag, curr_iter, _state_iter, _motion_start_iter); 
      //printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    //}
    //if(curr_iter == max_iter){ 
      //printf("flag:%d, curr iter: %lu, state iter: %llu, motion start iter: %d\n", 
        //_flag, curr_iter, _state_iter, _motion_start_iter); 
      //printf("inter pos: %f, %f, %f\n", inter_pos[0], inter_pos[1], inter_pos[2]);
    //}
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Jump5TypesMotionOPT<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_FRONTJUMPOPT:
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND: 
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_FRONTJUMP << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Jump5TypesMotionOPT<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Jump5TypesMotionOPT<T>::onExit() {
  // nothing to clean up?
    _data_reader->unload_control_plan();
    _stand_up_in_landing_yaw= false;
    _count=0;
    _count_yaw_stand=0;
}

// template class FSM_State_Jump5TypesMotionOPT<double>;
template class FSM_State_Jump5TypesMotionOPT<float>;
