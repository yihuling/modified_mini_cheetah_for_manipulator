#include "DataReader.hpp"
#include <Configuration.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

DataReader::DataReader(MIT_UserParameters* parameters) {
   _parameters=parameters;
//            load_data_index_from_yaml();
//          bool use_3d_data_flag = true;
//          if (use_3d_data_flag) {
//              load_control_plan(THIS_COM "config/jump_side_opt.dat", this->opt_3D_plan_cols);
//              printf("[3D Jump OPT Method DataReader] Setup for mini cheetah\n");
//          } else {THIS_COM "config/jumpmotiondata/jump_motion_library_index.yaml"
//              load_control_plan(THIS_COM "config/jump_front_opt.dat", this->opt_plan_cols);//jump_front_opt.dat
//              printf("[Front Jump OPT Method DataReader] Setup for mini cheetah\n");
//          }
}

void DataReader::load_control_plan(const char* filename) {
  printf("[Aerial Motions DataReader] Loading control plan %s...\n", filename);
  FILE* f = fopen(filename, "rb");
  if (!f) {
    printf("[Aerial Motions DataReader] Error loading control plan!\n");
    return;
  }
  fseek(f, 0, SEEK_END);
  uint64_t file_size = ftell(f);
  fseek(f, 0, SEEK_SET);

  printf("[Aerial Motions DataReader] Allocating %ld bytes for control plan\n",
         file_size);

  plan_buffer = (float*)malloc(file_size + 1);

  if (!plan_buffer) {
    printf("[Aerial Motions DataReader] malloc failed!\n");
    return;
  }

  uint64_t read_success = fread(plan_buffer, file_size, 1, f);
  if (!read_success) {
    printf("[Aerial Motions DataReader] Error: fread failed.\n");
  }
  if (file_size % sizeof(float)) {
    printf(
        "[Aerial Motions DataReader] Error: file size isn't divisible by size of "
        "float!\n");
  }
  fclose(f);

  plan_loaded = true;


    plan_timesteps = file_size / (sizeof(float) * _opt_plan_cols);
    printf("[2D JumpOpt DataReader] Done loading plan for %d timesteps\n",
           plan_timesteps);

}
bool DataReader::areEqual(const Vec3<float>& p1, const Vec3<float>& p2,float_t EPSILON) {
    return fabs(p1[0] - p2[0]) < EPSILON &&
           fabs(p1[1] - p2[1]) < EPSILON &&
           fabs(p1[2] - p2[2]) < EPSILON;
}
void DataReader::load_data_index_from_yaml(const char *yamlfilename)
{
    //stand for safety
    std::string motion_type[]={"front_motion","rear_motion","left_motion","right_motion","turn_yaw_motion","stand"};
    std::string f_j_o_type[]={"jump","flip","obstacle","stand"};
    YAML::Node config = YAML::LoadFile(yamlfilename);
    std::string motion_type_tmp;
    std::string flip_or_jump_or_bostacle;
    std::string motion_file_name;
    switch ((int)this->_parameters->target_com_info_2[2]) {
        case 1:
            motion_type_tmp=motion_type[0];
            break;
        case 2:
            motion_type_tmp=motion_type[1];
            break;
        case 3:
            motion_type_tmp=motion_type[2];
            break;
        case 4:
            motion_type_tmp=motion_type[3];
            break;
        case 5:
            motion_type_tmp=motion_type[4];
            break;
        default:
            motion_type_tmp=motion_type[5];
            break;

    }

    switch ((int)this->_parameters->target_com_info_2[1]) { //0 :jump 1:flip 2:obstacle
        case 0:
            flip_or_jump_or_bostacle=f_j_o_type[0];
            break;
        case 1:
            flip_or_jump_or_bostacle=f_j_o_type[1];
            break;
        case 2:
            flip_or_jump_or_bostacle=f_j_o_type[2];
            break;
        default:
            flip_or_jump_or_bostacle=f_j_o_type[0];
            break;

    }
    if (motion_type_tmp!="stand" && flip_or_jump_or_bostacle!="stand") {
        for (const auto &p : config[motion_type_tmp]) {
            for (const auto &key_value : p) {
                YAML::Node key = key_value.first;
                YAML::Node value = key_value.second;
                std::string s = key.as<std::string>();
                if (s == flip_or_jump_or_bostacle) {
                    for (int i = 0; i < int(value.size()); i++) {
                        Vec3<float> p_cmd, p_real;
                        p_cmd<< value[i]["pos_d"][0].as<float>(), value[i]["pos_d"][1].as<float>(), value[i]["pos_d"][2].as<float>();
                        p_real<< (float) this->_parameters->target_com_info_1[0], (float) this->_parameters->target_com_info_1[1], (float) this->_parameters->target_com_info_1[2];
                        if (areEqual(p_cmd, p_real, (float_t) this->_parameters->eucliddean_distance_radius[0])) {
                            this->motion_type_name[_count_path] = value[i]["filename"].as<std::string>();
                            _count_path+=1;
                            best_energy.push_back(value[i]["optimal_energy_cost"].as<float>());
                        }
                    }
                }
            }
        }
//        std::cout<<"_count_path "<<_count_path<<std::endl;
        if (_count_path>0)
        {
            auto smallest = std::min_element(std::begin(best_energy), std::end(best_energy));
            motion_file_name=this->motion_type_name[std::distance(std::begin(best_energy), smallest)];

        } else
        {
            motion_file_name=this->motion_type_name[0];
        }
        //step2: if more than one option
        std::cout << "[Robot will do] : " << motion_type_tmp << " " << flip_or_jump_or_bostacle << std::endl;
        if (motion_file_name.empty())
        {
            std::cout<<"No solution for this target,please check your desire pos"<<std::endl;
            estop_for_motions= true;
        } else {
            load_control_plan(
                    (std::string(THIS_COM "config/jumpmotiondata/") + std::string(motion_type_tmp) + std::string("/") +
                     std::string(flip_or_jump_or_bostacle) + std::string("/") +
                     std::string(motion_file_name)).c_str());
        }
    } else
    {
        std::cout<<"It's not safe for this option,please check your param"<<std::endl;
        estop_for_motions= true;
    }
    }

float* DataReader::get_plan_at_time(int timestep) {
  if (!plan_loaded) {
    printf(
        "[Aerial Motions DataReader] Error: get_plan_at_time called without a "
        "plan!\n");
    return nullptr;
  }

  if (timestep < 0 || timestep >= plan_timesteps) {
    printf(
        "[Aerial Motions DataReader] Error: get_plan_at_time called for timestep %d\n"
        "\tmust be between 0 and %d\n",
        timestep, plan_timesteps - 1);
    timestep = plan_timesteps - 1;
    // return nullptr; // TODO: this should estop the robot, can't really
    // recover from this!
  }
        return plan_buffer + _opt_plan_cols * timestep;

}

void DataReader::unload_control_plan() {
  free(plan_buffer);
  plan_timesteps = -1;
  plan_loaded = false;
  _count_path=0;
  printf("[Aerial Motions DataReader] Unloaded plan.\n");
}
