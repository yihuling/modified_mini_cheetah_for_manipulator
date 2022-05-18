#include <iostream>
#include <string>
#include "forwardKinematics.h"

using namespace std;
using namespace pinocchio;

Eigen::Matrix3d get_the_sixth_joint_rotation_matrix(const Eigen::Vector6d &q)
{
	const string urdf_filename = (argc <= 1) ? PINOCCHIO_MODEL_DIR + string("/catkin_ws/src/six_dof_arm_hkclr/urdf/six_dof_arm_hkclr.urdf") : argv[1];

	//load the urdf model
	Model model;
	pinocchio::urdf::buildModel(urdf_filename, model);
	//cout << "model name :" << model.name << endl;

	//Create the data required by the algorithms
	Data data(model);

	//Sample a random configuration
	//Eigen::VectorXd q = randomConfiguration(model);
	//Eigen::VectorXd q(model.nv);
	//q << -0.5, -0.4, -0.6, 0.5, -1, -1;
	//cout << "q:" << q.transpose() << endl;

	//Perform the forward kinematics over the kinematic tree
	forwardKinematics(model, data, q);

	//print out the placement of each joint of the kinematic tree
	Eigen::Matrix3d res;
	res = data.oMi[(JointIndex)6].rotation();
	return res;
	//for (JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
	//{
	//	cout << setw(24) << left << model.names[joint_id] << ":" << endl << "rotation_matrix:" << endl
	//		<< fixed << setprecision(5)
	//		<< data.oMi[joint_id].rotation().transpose() << endl << "position:" << endl << data.oMi[joint_id].translation().transpose() << endl;
	//}
	//cout << "model.nv : " << model.nv << "model.njoints :" << model.njoints << endl;

}

Eigen::Vector6d get_the_sixth_joint_translation_matrix(const Eigen::Vector6d &q)
{
	const string urdf_filename = (argc <= 1) ? PINOCCHIO_MODEL_DIR + string("/catkin_ws/src/six_dof_arm_hkclr/urdf/six_dof_arm_hkclr.urdf") : argv[1];

	//load the urdf model
	Model model;
	pinocchio::urdf::buildModel(urdf_filename, model);
	//cout << "model name :" << model.name << endl;

	//Create the data required by the algorithms
	Data data(model);

	//Perform the forward kinematics over the kinematic tree
	forwardKinematics(model, data, q);

	//print out the placement of each joint of the kinematic tree
	Eigen::Vector6d res;
	res = data.oMi[(JointIndex)6].translation();
	return res;
}
