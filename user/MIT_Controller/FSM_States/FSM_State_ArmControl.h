#ifndef FSM_STATE_ARMCONTROL_H
#define FSM_STATE_ARMCONTROL_H
#include "FSM_State.h"
#include "inverseKinematics.h"
#include "forwardKinematics.h"

/**
 *
 */
template <typename T>
class FSM_State_ArmControl : public FSM_State<T> {
public:
    FSM_State_ArmControl(ControlFSMData<T>* _controlFSMData);

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
    std::vector< Vec3<T> > _ini_foot_pos;
    std::vector< Vec3<T> > _ini_joint_pos;
    std::vector< Vec3<T> > _end_joint_pos;
};

#endif  // FSM_STATE_SITDOWN_H
