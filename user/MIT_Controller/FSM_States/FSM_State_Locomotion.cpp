/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
//#include <rt/rt_interface_lcm.h>
#include <fstream>
/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  if(_controlFSMData->_quadruped->_robotType == RobotType::MINI_CHEETAH){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
        //30 / (1000. * _controlFSMData->controlParameters->controller_dt),
        //22 / (1000. * _controlFSMData->controlParameters->controller_dt),
        30/ (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else if(_controlFSMData->_quadruped->_robotType == RobotType::CHEETAH_3){
    cMPCOld = new ConvexMPCLocomotion(_controlFSMData->controlParameters->controller_dt,
//        33 / (1000. * _controlFSMData->controlParameters->controller_dt),
        25 / (1000. * _controlFSMData->controlParameters->controller_dt),
        _controlFSMData->userParameters);

  }else{
    assert(false);
  }


//  this->turnOnAllSafetyChecks();
   this-> turnOffAllSafetyChecks();////////////////////////////////////////////////////////////////////////////
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
}

template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  cMPCOld->initialize();
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
  printf("[FSM LOCOMOTION] On Enter\n");
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */

template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();
}

extern rc_control_settings rc_control;

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition() {
  // Get the next state
  iter++;

  // Switch FSM control mode
  if(locomotionSafe())
       {
    switch ((int)this->_data->controlParameters->control_mode) {
      case K_LOCOMOTION:
        break;

      case K_BALANCE_STAND:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::BALANCE_STAND;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_PASSIVE:
        // Requested change to BALANCE_STAND
        this->nextStateName = FSM_StateName::PASSIVE;

        // Transition time is immediate
        this->transitionDuration = 0.0;

        break;

      case K_STAND_UP:
        this->nextStateName = FSM_StateName::STAND_UP;
        this->transitionDuration = 0.;
        break;

      case K_RECOVERY_STAND:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        this->transitionDuration = 0.;
        break;

      case K_VISION:
        this->nextStateName = FSM_StateName::VISION;
        this->transitionDuration = 0.;
        break;

      default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                  << K_LOCOMOTION << " to "
                  << this->_data->controlParameters->control_mode << std::endl;
    }
  } else {
    this->nextStateName = FSM_StateName::RECOVERY_STAND;
    this->transitionDuration = 0.;
    rc_control.mode = RC_mode::RECOVERY_STAND;
    printf("BILLCHEN RINTF: locomotion force to RECOVERY_STAND\n");
  }


  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::BALANCE_STAND:
      LocomotionControlStep();

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
          printf("BILLCHEN RINTF: transitionDuration iter: %d\t%d\n",iter, int(this->transitionDuration*1000));
      } else {
        this->transitionData.done = false;
      }

      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
          printf("BILLCHEN RINTF: transitionData.done = true \n");
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() {
  auto& seResult = this->_data->_stateEstimator->getResult();

  const T max_roll = 80;//40;
  const T max_pitch = 80;//40;

  if(std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll)) {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if(std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch)) {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for(int leg = 0; leg < 4; leg++) {
    auto p_leg = this->_data->_legController->datas[leg].p;
    if(p_leg[2] > 0) {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if(std::fabs(p_leg[1] > 0.28))//0.18))
    {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->_legController->datas[leg].v.norm();
    if(std::fabs(v_leg) > 19.) {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;

}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  // estimateContact();
    auto& seResult =this->_data->_stateEstimator->getResult();
    _curr_time += this->_data->controlParameters->controller_dt;
  cMPCOld->run<T>(*this->_data);
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for(int leg(0); leg<4; ++leg){
    pDes_backup[leg] = this->_data->_legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->_legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->_legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->_legController->commands[leg].kdCartesian;
  }

  if(this->_data->userParameters->use_wbc > 0.9){
    _wbc_data->pBody_des = cMPCOld->pBody_des;
    _wbc_data->vBody_des = cMPCOld->vBody_des;
    _wbc_data->aBody_des = cMPCOld->aBody_des;

    _wbc_data->pBody_RPY_des = cMPCOld->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld->vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i){
      _wbc_data->pFoot_des[i] = cMPCOld->pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld->vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld->aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld->Fr_des[i]; 
    }
    _wbc_data->contact_state = cMPCOld->contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }
  for(int leg(0); leg<4; ++leg){
    //this->_data->_legController->commands[leg].pDes = pDes_backup[leg];
    this->_data->_legController->commands[leg].vDes = vDes_backup[leg];
    //this->_data->_legController->commands[leg].kpCartesian = Kp_backup[leg];
    this->_data->_legController->commands[leg].kdCartesian = Kd_backup[leg];
  }

    static std::ofstream log_trot("./log_trot.txt");//

    log_trot<< _curr_time<<" ";//1
    log_trot<< this->_data->_legController->datas[0].q[0]<<" ";//2
    log_trot<< this->_data->_legController->datas[0].q[1]<<" ";//3
    log_trot<< this->_data->_legController->datas[0].q[2]<<" ";//4

    log_trot<< this->_data->_legController->datas[0].qd[0]<<" ";//5
    log_trot<< this->_data->_legController->datas[0].qd[1]<<" ";//6
    log_trot<< this->_data->_legController->datas[0].qd[2]<<" ";//7

    log_trot<< this->_data->_legController->datas[0].tauEstimate[0]<<" ";//8
    log_trot<< this->_data->_legController->datas[0].tauEstimate[1]<<" ";//9
    log_trot<< this->_data->_legController->datas[0].tauEstimate[2]<<" ";//10
    log_trot<< this->_data->_legController->datas[0].p[0]<<" ";//11
    log_trot<< this->_data->_legController->datas[0].p[1]<<" ";//12
    log_trot<< this->_data->_legController->datas[0].p[2]<<" ";//13

    log_trot<< this->_data->_legController->datas[3].q[0]<<" ";//14
    log_trot<< this->_data->_legController->datas[3].q[1]<<" ";//15
    log_trot<< this->_data->_legController->datas[3].q[2]<<" ";//16

    log_trot<< this->_data->_legController->datas[3].qd[0]<<" ";//17
    log_trot<< this->_data->_legController->datas[3].qd[1]<<" ";//18
    log_trot<< this->_data->_legController->datas[3].qd[2]<<" ";//19

    log_trot<< this->_data->_legController->datas[3].tauEstimate[0]<<" ";//20
    log_trot<< this->_data->_legController->datas[3].tauEstimate[1]<<" ";//21
    log_trot<< this->_data->_legController->datas[3].tauEstimate[2]<<" ";//22

    log_trot<< this->_data->_legController->datas[3].p[0]<<" ";//23
    log_trot<< this->_data->_legController->datas[3].p[1]<<" ";//24
    log_trot<< this->_data->_legController->datas[3].p[2]<<" ";//25

    log_trot<< this->_data->_legController->datas[1].q[0]<<" ";//26
    log_trot<< this->_data->_legController->datas[1].q[1]<<" ";//27
    log_trot<< this->_data->_legController->datas[1].q[2]<<" ";//28

    log_trot<< this->_data->_legController->datas[1].qd[0]<<" ";//29
    log_trot<< this->_data->_legController->datas[1].qd[1]<<" ";//30
    log_trot<< this->_data->_legController->datas[1].qd[2]<<" ";//31

    log_trot<< this->_data->_legController->datas[1].tauEstimate[0]<<" ";//32
    log_trot<< this->_data->_legController->datas[1].tauEstimate[1]<<" ";//33
    log_trot<< this->_data->_legController->datas[1].tauEstimate[2]<<" ";//34

    log_trot<< this->_data->_legController->datas[1].p[0]<<" ";//35
    log_trot<< this->_data->_legController->datas[1].p[1]<<" ";//36
    log_trot<< this->_data->_legController->datas[1].p[2]<<" ";//37

    log_trot<< this->_data->_legController->datas[2].q[0]<<" ";//38
    log_trot<< this->_data->_legController->datas[2].q[1]<<" ";//39
    log_trot<< this->_data->_legController->datas[2].q[2]<<" ";//40

    log_trot<< this->_data->_legController->datas[2].qd[0]<<" ";//41
    log_trot<< this->_data->_legController->datas[2].qd[1]<<" ";//42
    log_trot<< this->_data->_legController->datas[2].qd[2]<<" ";//43

    log_trot<< this->_data->_legController->datas[2].tauEstimate[0]<<" ";//44
    log_trot<< this->_data->_legController->datas[2].tauEstimate[1]<<" ";//45
    log_trot<< this->_data->_legController->datas[2].tauEstimate[2]<<" ";//46

    log_trot<< this->_data->_legController->datas[2].p[0]<<" ";//47
    log_trot<< this->_data->_legController->datas[2].p[1]<<" ";//48
    log_trot<< this->_data->_legController->datas[2].p[2]<<" ";//49

    log_trot<<this->_data->_legController->commands[0].qDes[0]<<" ";//50
    log_trot<<this->_data->_legController->commands[0].qDes[1]<<" ";//51
    log_trot<<this->_data->_legController->commands[0].qDes[2]<<" ";//52

    log_trot<<this->_data->_legController->commands[1].qDes[0]<<" ";//53
    log_trot<<this->_data->_legController->commands[1].qDes[1]<<" ";//54
    log_trot<<this->_data->_legController->commands[1].qDes[2]<<" ";//55
    log_trot<<seResult.position[0]<<" ";//56
    log_trot<<seResult.position[1]<<" ";//57
    log_trot<<seResult.position[2]<<" ";//58
    log_trot<<seResult.rpy[0]<<" ";//59
    log_trot<<seResult.rpy[1]<<" ";//60
    log_trot<<seResult.rpy[2]<<" ";//61
    log_trot<<seResult.omegaWorld[0]<<" ";//62
    log_trot<<seResult.omegaWorld[1]<<" ";//63
    log_trot<<seResult.omegaWorld[2]<<" ";//64
    log_trot<<seResult.vWorld[0]<<" ";//65
    log_trot<<seResult.vWorld[1]<<" ";//66
    log_trot<<seResult.vWorld[2]<<" ";//67
    log_trot<<this->_data->_legController->commands[0].pDes[0]<<" "<<this->_data->_legController->commands[0].pDes[1]<<" "<<this->_data->_legController->commands[0].pDes[2]<<" ";//68-69-70
    log_trot<<this->_data->_legController->commands[1].pDes[0]<<" "<<this->_data->_legController->commands[1].pDes[1]<<" "<<this->_data->_legController->commands[1].pDes[2]<<" ";//71-72-73
    log_trot<<this->_data->_legController->commands[0].vDes[0]<<" "<<this->_data->_legController->commands[0].vDes[1]<<" "<<this->_data->_legController->commands[0].vDes[2]<<" ";//74-75-76
    log_trot<<this->_data->_legController->commands[1].vDes[0]<<" "<<this->_data->_legController->commands[1].vDes[1]<<" "<<this->_data->_legController->commands[1].vDes[2]<<" ";//77-78-79
    log_trot<<this->_data->_legController->datas[0].v[0]<<" "<<this->_data->_legController->datas[0].v[1]<<" "<<this->_data->_legController->datas[0].v[2]<<" ";//80-81-82
    log_trot<<this->_data->_legController->datas[1].v[0]<<" "<<this->_data->_legController->datas[1].v[1]<<" "<<this->_data->_legController->datas[1].v[2]<<" ";//83-84-85

    log_trot<<std::endl;
}

/**
 * Stance leg logic for impedance control. Prevent leg slipping and
 * bouncing, as well as tracking the foot velocity during high speeds.
 */
template <typename T>
void FSM_State_Locomotion<T>::StanceLegImpedanceControl(int leg) {
  // Impedance control for the stance leg
  this->cartesianImpedanceControl(
      leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),
      this->_data->controlParameters->stand_kp_cartesian,
      this->_data->controlParameters->stand_kd_cartesian);
}

// template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;
