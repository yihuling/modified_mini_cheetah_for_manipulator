#ifndef FORWARDKINEMATICS
#define FORAWARDKINEMATICS

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"


#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/yihu"
#endif

Eigen::Matrix3d get_the_sixth_joint_rotation_matrix(const Eigen::Vector6d &q);
Eigen::Vector6d get_the_sixth_joint_translation_matrix(const Eigen::Vector6d &q);
#endif
