/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */
#include "Math/Interpolation.h"
#include "FSM_State_ArmControl.h"
#define PI (3.141592654f)
 /**
  * Constructor for the FSM State that passes in state specific info to
  * the generic FSM State constructor.
  *
  * @param _controlFSMData holds all of the relevant control data
  */
template <typename T>
FSM_State_ArmControl<T>::FSM_State_ArmControl(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::ARMCONTROL, "ARMCONTROL"),
    _ini_joint_pos(4), _end_joint_pos(4) {
    // Do nothing
    // Set the pre controls safety checks
    this->checkSafeOrientation = false;

    // Post control safety checks
    this->checkPDesFoot = false;
    this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_ArmControl<T>::onEnter() {
    // Default is to not transition
    this->nextStateName = this->stateName;

    // Reset the transition data
    this->transitionData.zero();

    // Reset iteration counter
    iter = 0;

    /*----------------------------modified initial position for our manipulator--------------------------------------------------------
    float l1 = this->_data->_quadruped->_hipLinkLength;
    float l2 = this->_data->_quadruped->_kneeLinkLength;

    //求站起来时，hip与knee角度
    float h = 0.29;
    float theta1 = -acosf((l1 * l1 + h * h - l2 * l2) / (2 * l1 * h));
    float theta2 = PI - acosf((l1 * l1 + l2 * l2 - h * h) / (2 * l1 * l2));

    for (size_t leg(0); leg < 4; ++leg) {

        _ini_joint_pos[leg][0] = 0;
        _ini_joint_pos[leg][1] = theta1;
        _ini_joint_pos[leg][2] = theta2;

        _end_joint_pos[leg][0] = 0;
        _end_joint_pos[leg][1] = -1.2596790678968424f;//-1.2596790678968424 2.8019000774224456
        _end_joint_pos[leg][2] = 2.8019000774224456f;
    }
}
-----------------------------------------------------------------------------------------------------------------------------*/
/*-----------CAN1-----1关节和2关节的起始位置----------------------------------*/
    _ini_joint_pos[0][0] = 0.0f;
    _ini_joint_pos[0][1] = 0.0f;
    _ini_joint_pos[0][2] = 0.0f;
    /*-----------CAN1-----1关节和2关节的终点位置----------------------------------*/
    _end_joint_pos[0][0] = 0.785f;
    _end_joint_pos[0][1] = -0.785f;
    _end_joint_pos[0][2] = 0.0f;
    /*-----------CAN2-----3关节和4关节的起始位置----------------------------------*/
    _ini_joint_pos[1][0] = 0.0f;
    _ini_joint_pos[1][1] = 0.0f;
    _ini_joint_pos[1][2] = 0.0f;
    /*-----------CAN2-----3关节和4关节的终点位置----------------------------------*/
    _end_joint_pos[1][0] = 0.0f;
    _end_joint_pos[1][1] = 1.57f;
    _end_joint_pos[1][2] = 0.0f;
    /*-----------CAN3-----5关节和6关节的起始位置----------------------------------*/
    _ini_joint_pos[2][0] = 0.0f;
    _ini_joint_pos[2][1] = 0.0f;
    _ini_joint_pos[2][2] = 0.0f;
    /*-----------CAN3-----5关节和6关节的终点位置----------------------------------*/
    _end_joint_pos[2][0] = 0.785f;
    _end_joint_pos[2][1] = 1.57f;
    _end_joint_pos[2][2] = 0.0f;
    /*-----------CAN4-----7关节和8关节的起始位置----------------------------------*/
    _ini_joint_pos[3][0] = 0.0f;
    _ini_joint_pos[3][1] = 0.0f;
    _ini_joint_pos[3][2] = 0.0f;
    /*-----------CAN4-----7关节和8关节的终点位置----------------------------------*/
    _end_joint_pos[3][0] = 0.0f;
    _end_joint_pos[3][1] = 0.0f;
    _end_joint_pos[3][2] = 0.0f;
}
/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_ArmControl<T>::run() {

    if (this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
        Vec4<T> contactState;
        contactState << 0.5, 0.5, 0.5, 0.5;
        this->_data->_stateEstimator->setContactPhase(contactState);
        iter++;
        T sit_down_time = 1.5;
        T t = (iter * 0.002f) / sit_down_time;

        if (t > 1.) { t = 1.; }

        Vec3<T> kp(80,
            80,
            80);
        Vec3<T> kd(2,
            2,
            2);
        for (int leg = 0; leg < 4; leg++)
        {
            this->_data->_legController->commands[leg].kpJoint = kp.asDiagonal();
            this->_data->_legController->commands[leg].kdJoint = kd.asDiagonal();
            //数据插值
            this->_data->_legController->commands[leg].qDes
                = Interpolate::cubicBezier<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t);
            this->_data->_legController->commands[leg].qdDes
                = Interpolate::cubicBezierFirstDerivative<Vec3<T>>(_ini_joint_pos[leg], _end_joint_pos[leg], t) / sit_down_time;

        }

    }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_ArmControl<T>::checkTransition() {
    this->nextStateName = this->stateName;
    iter++;

    // Switch FSM control mode
    switch ((int)this->_data->controlParameters->control_mode) {
    case K_ARMCONTROL:
        break;
    case K_PASSIVE:
        this->nextStateName = FSM_StateName::PASSIVE;
        break;
    default:
        std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
            << K_PASSIVE << " to "
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
TransitionData<T> FSM_State_ArmControl<T>::transition() {
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

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_ArmControl<T>::onExit() {
    // Nothing to clean up when exiting
}

// template class FSM_State_SitDown<double>;
template class FSM_State_ArmControl<float>;
