#include "Jump5TypesMotionOPT.hpp"


template <typename T>
Jump5TypesMotionOPT<T>::Jump5TypesMotionOPT(DataReader* data_reader, float _dt) : DataReadCtrl<T>(data_reader, _dt) {}


template <typename T>
Jump5TypesMotionOPT<T>::~Jump5TypesMotionOPT() {}

template <typename T>
void Jump5TypesMotionOPT<T>::OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command) {
  DataCtrl::_state_machine_time = _curr_time - DataCtrl::_ctrl_start_time;

  DataCtrl::_b_Preparation = b_preparation;
  _update_joint_command();

  for (int leg = 0; leg < 4; ++leg) {
    for (int jidx = 0; jidx < 3; ++jidx) {
        command[leg].tauFeedForward[jidx] = DataCtrl::_jtorque[3 * leg + jidx];
        command[leg].qDes[jidx] = DataCtrl::_des_jpos[3 * leg + jidx];
        command[leg].qdDes[jidx] = DataCtrl::_des_jvel[3 * leg + jidx];
        command[leg].kpJoint(jidx, jidx) = DataCtrl::_Kp_joint[jidx];
        command[leg].kdJoint(jidx, jidx) = DataCtrl::_Kd_joint[jidx];
    }
  }
//    std::cout<<"joint in opt"<<command[0].qDes[1] <<" "<<command[0].qDes[2]<<" "<<command[3].qDes[1]
//             <<" "<<command[3].qDes[2]<< std::endl;
}

template <typename T>
void Jump5TypesMotionOPT<T>::_update_joint_command() {
    int pre_mode_duration(500);//four foot jump 1200
    float tau_mult;
    DataCtrl::_des_jpos.setZero();
    DataCtrl::_des_jvel.setZero();
    DataCtrl::_jtorque.setZero();
//  DataCtrl::_data_reader->_parameters->motion_jump_kp;
//  std::cout<<"kp_joint--->"<<DataCtrl::_data_reader->_parameters->motion_jump_kp<<std::endl;
    DataCtrl::_Kp_joint[0] = DataCtrl::_data_reader->_parameters->motion_jump_kp[0];//={180.0, 180.0, 280.0};
    DataCtrl::_Kp_joint[1] = DataCtrl::_data_reader->_parameters->motion_jump_kp[1];//={180.0, 180.0, 280.0};
    DataCtrl::_Kp_joint[2] = DataCtrl::_data_reader->_parameters->motion_jump_kp[2];//={180.0, 180.0, 280.0};

    DataCtrl::_Kd_joint[0] = DataCtrl::_data_reader->_parameters->motion_jump_kd[0];// {1.8,2.8,2.8};
    DataCtrl::_Kd_joint[1] = DataCtrl::_data_reader->_parameters->motion_jump_kd[1];// {1.8,2.8,2.8};
    DataCtrl::_Kd_joint[2] = DataCtrl::_data_reader->_parameters->motion_jump_kd[2];// {1.8,2.8,2.8};
//    this->_Kp_joint = {50.0, 50.0, 50.0};//{15.0, 15.0, 15.0};
//    this->_Kd_joint = {1.8, 1.8, 1.8};
    // OBTAIN TIMSTEP DATA FROM THE DATA FILE
    if (DataCtrl::current_iteration > DataCtrl::_data_reader->plan_timesteps - 1) { //执行完停留在最后时刻
        DataCtrl::current_iteration = DataCtrl::_data_reader->plan_timesteps - 1;

    }
    curr_iter_for_plot = DataCtrl::current_iteration;
//    float tau_mult = 0.0;
//    printf("%d:[iter] current\n\r",DataCtrl::current_iteration);
// OBTAIN DATA FROM THE JUMP_DATA FILE GENERATED IN MATLAB

    float *current_step = DataCtrl::_data_reader->get_plan_at_time(DataCtrl::current_iteration); //每次取24个数据出来
//  float* tau = current_step + tau_offset;
//   std::cout<<"Time "<<DataCtrl::current_iteration<<std::endl;
//   for (int i = 0; i < 36; ++i) {
//        printf("  [%d:] %.2f ",i,current_step[i]);
//    }
//    printf("\n\r");
//   printf("tau----\n\r");
//  float* tau = current_step + tau_offset;
//    for (int i = 0; i < 22; ++i) {
//        printf(" %.2f ",tau[i]);
//    }
//    printf("\n\r");
    if ((DataCtrl::pre_mode_count < pre_mode_duration) || DataCtrl::_b_Preparation) {
        // move to the initial configuration to prepare for
        // backfliping
        if (DataCtrl::pre_mode_count == 0) {
            printf("plan_timesteps: %d \n", DataCtrl::_data_reader->plan_timesteps);
        }
        //printf("pre_mode_count: %d \n", DataCtrl::pre_mode_count);

        DataCtrl::pre_mode_count += DataCtrl::_key_pt_step;
        DataCtrl::current_iteration = 0;
        tau_mult = 0.0;
    } else {
        tau_mult = 1.0f;
    }
    Vec12<float> q_des;
    Vec12<float> qd_des;

    Vec3<float> tau_max;


    float max_angular_velocity=27.0;//motor max velocity
    /*
     * lz from HK 20220209
       q_des= [0: leg_0_q0,1:leg_0_q1,2:leg_0_q2,3:leg_1_q0,4:leg_1_q1,5:leg_1_q2,6:leg_2_q0,7:leg_2_q1,8:leg_2_q2,9:leg_3_q0,10:leg_3_q1,11:leg_3_q2,]';
       qd_des = [12:leg_0_qd0,13:leg_0_qd1,14:leg_0_qd2,15:leg_1_qd0,16:leg_1_qd1,17:leg_1_qd2,18:leg_2_qd0,19:leg_2_qd1,20:leg_2_qd2,21:leg_3_qd0,22:leg_3_qd1,23:leg_3_qd2,];
     * */
    for (int i = 0; i < 24; i++)
    {
        if (i<12) {
            q_des[i] = current_step[i];
        } else
        {
            if (current_step[i]<0.0 && std::abs(current_step[i])>max_angular_velocity)
            {
                qd_des[i-12]=-max_angular_velocity;
            } else if (current_step[i]>0.0 && std::abs(current_step[i])>max_angular_velocity)
            {
                qd_des[i-12]=max_angular_velocity;
            } else {
                qd_des[i - 12] = current_step[i];
            }
        }
    }
    tau_max << tau_offset*tau_mult, tau_offset*tau_mult,tau_offset*tau_mult;

    float s(0.);
    bool jump_flag_initial= true;

    float jump_last_index=200;//if yaw and rear four leg jump use 250
    if (DataCtrl::current_iteration >= DataCtrl::_data_reader->plan_timesteps - jump_last_index) {  // ramp to landing configuration
        q_des << 0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0;
        qd_des << 0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0;
        tau_max << 0.0, 0.0, 0.0;



        Vec12<float> q_des_0;
        Vec12<float> q_des_f;

        current_step = DataCtrl::_data_reader->get_plan_at_time(DataCtrl::_data_reader->plan_timesteps - jump_last_index);

        for (int i = 0; i < 12; i++)
        {q_des_0[i] = current_step[i];}
        current_step = DataCtrl::_data_reader->get_plan_at_time(0);
        if(jump_flag_initial) { //jump
            s = 0.8;
            if (DataCtrl::_data_reader->_parameters->target_com_info_2[2]==3 && DataCtrl::_data_reader->_parameters->target_com_info_2[1]==0) {
                q_des_f << 0.0, -1.1349009505576744, 2.3940083039207725, //0
                        0.6, -1.1349009505576744, 2.3940083039207725, //1
                        0.0, -1.1349009505576744, 2.3940083039207725, //2
                        0.6, -1.1349009505576744, 2.3940083039207725; //3
                q_des = (1 - s) * q_des_0 + s * q_des_f;
                this->_Kp_joint = {20.0, 20.0, 40.0};
                this->_Kd_joint = {1.2, 2.0, 2.0};
            }else if(DataCtrl::_data_reader->_parameters->target_com_info_2[2]==4 && DataCtrl::_data_reader->_parameters->target_com_info_2[1]==0)
            {
                q_des_f << -0.8, -1.1349009505576744, 2.3940083039207725, //0
                        0., -1.1349009505576744, 2.3940083039207725, //1
                        -0.8, -1.1349009505576744, 2.3940083039207725, //2
                        0.0, -1.1349009505576744, 2.3940083039207725; //3
                q_des = (1 - s) * q_des_0 + s * q_des_f;
                this->_Kp_joint = {20.0, 20.0, 40.0};
                this->_Kd_joint = {1.2, 2.0, 2.0};
            } else if(DataCtrl::_data_reader->_parameters->target_com_info_2[2]==2 && DataCtrl::_data_reader->_parameters->target_com_info_2[1]==0)
            {
                q_des_f << 0., -1.1349009505576744, 2.3940083039207725, //0
                        0., -1.1349009505576744, 2.3940083039207725, //1
                        -0., -2.349009505576744, 2.3940083039207725, //2
                        0.0, -2.349009505576744, 2.3940083039207725; //3
                q_des = (1 - s) * q_des_0 + s * q_des_f;
                this->_Kp_joint = {20.0, 80.0, 40.0};
                this->_Kd_joint = {1.2, 2.0, 2.0};
            } else if(DataCtrl::_data_reader->_parameters->target_com_info_2[2]==3 && DataCtrl::_data_reader->_parameters->target_com_info_2[1]==1)
            {
                q_des_f << 0.0, -1.1349009505576744, 2.3940083039207725, //0
                        0.8, -1.1349009505576744, 2.3940083039207725, //1
                        0.0, -1.1349009505576744, 2.3940083039207725, //2
                        0.8, -1.1349009505576744, 2.3940083039207725; //3
                q_des = (1 - s) * q_des_0 + s * q_des_f;
                this->_Kp_joint = {20.0, 20.0, 40.0};
                this->_Kd_joint = {1.2, 2.0, 2.0};
            }else if(DataCtrl::_data_reader->_parameters->target_com_info_2[2]==1 && DataCtrl::_data_reader->_parameters->target_com_info_2[1]==2)
            {
                //jump to desk change the param back
//                q_des_f << 0.0, -1.1349009505576744, 2.3940083039207725, //0
//                        0.0, -1.1349009505576744, 2.3940083039207725, //1
//                        0.0, -1.1349009505576744, 2.3940083039207725, //2
//                        0.0, -1.1349009505576744, 2.3940083039207725; //3
                q_des_f << 0.0, 0.8349120829579479,0.3308607995092896, //0
                        0.0, 0.834920829579479,0.3308607995092896, //1
                        0.0, -1.1349009505576744, 2.3940083039207725, //2
                        0.0, -1.1349009505576744, 2.3940083039207725; //3
                q_des = (1 - s) * q_des_0 + s * q_des_f;
                this->_Kp_joint = {20.0, 20.0, 25.0};
                this->_Kd_joint = {1.2, 1.2, 1.2};
            }
            else
            {
                q_des_f << 0.0, -1.1349009505576744, 2.3940083039207725, //0
                        0.0, -1.1349009505576744, 2.3940083039207725, //1
                        0.0, -1.1349009505576744, 2.3940083039207725, //2
                        0.0, -1.1349009505576744, 2.3940083039207725; //3
                q_des = (1 - s) * q_des_0 + s * q_des_f;
                this->_Kp_joint = {20.0, 20.0, 40.0};
                this->_Kd_joint = {1.2, 2.0, 2.0};
            }

        } else//yaw turn
        {
                q_des_f << -0.8, -1.0, 2.715,
                0.8, -1.0, 2.715,
                -0.8, -1.0, 2.715,
                0.8, -1.0, 2.715;
                s=s+1;
            q_des =q_des_f;//q_des_0;//(1 - s) * q_des_0 + s * q_des_f;
            this->_Kp_joint = {20.0, 300.0, 360.0};
            this->_Kd_joint = {1.2, 4.0, 4.2};
        }

        //printf("tuck_iteration\n");

    }

        //1----0
        //3----2
        for (int leg = 0; leg < 4; ++leg) {
            for (int jidx = 0; jidx < 3; ++jidx) {
                DataCtrl::_jtorque[3 * leg + jidx]=tau_max[jidx];
                DataCtrl::_des_jpos[3 * leg + jidx]=q_des[3 * leg + jidx];
                DataCtrl::_des_jvel[3 * leg + jidx]=qd_des[3 * leg + jidx];
            }
        }

  // Update rate 0.5kHz
  DataCtrl::current_iteration += DataCtrl::_key_pt_step;

}



template class Jump5TypesMotionOPT<double>;
template class Jump5TypesMotionOPT<float>;
