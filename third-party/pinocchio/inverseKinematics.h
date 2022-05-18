#ifndef INVERSEKINEMATICS
#define INVERSEKINEMATICS

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/home/yihu"
#endif

Eigen::VectorXd get_the_inverse_solution_of_manipulator(const pinocchio::SE3 &oMdes);
#endif