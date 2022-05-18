#include <iostream>
#include <string>
#include "inverseKinematics.h"

using namespace std;
using namespace pinocchio;
using namespace Eigen;

Eigen::VectorXd get_the_inverse_solution_of_manipulator(const pinocchio::SE3 &oMdes)
{
	/*--------load the model and create the data structure of model-----------*/
	const string urdf_filename = (argc <= 1) ? PINOCCHIO_MODEL_DIR + string("/catkin_ws/src/six_dof_arm_hkclr/urdf/six_dof_arm_hkclr.urdf") : argv[1];
	Model model;
	pinocchio::urdf::buildModel(urdf_filename, model);
	cout << "the name of model : " << model.name << endl;
	Data data(model);
	/*--------define the pre-parameters required by the coming algorithm-------*/
	const double eps = 1e-4;
	const int IT_MAX = 1000;
	const double DT = 1e-1;
	const double damp = 1e-6;
	bool success = false;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;
	Vector6d err;
	Eigen::VectorXd v(model.nv);
	/*------define the end_joint id and the desired configuration of the end joint------*/
	const int JOINT_ID = 6;
	/*Eigen::Matrix3d m;
	m << 0.04034, 0.86350, 0.50273, -0.79678, -0.27580, 0.53766, 0.60293, -0.42225, 0.67690;*/
	//const pinocchio::SE3 oMdes(m, Eigen::Vector3d(0.04456, 0.06428, 0.39492));
	/*------------define the start configuration of the manipulator, it's a column vector--------*/
	//Eigen::VectorXd q = pinocchio::neutral(model);
	Eigen::VectorXd q(model.nv);
	q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	/*-------------------------------------------------------------------------------------------*/
	//create a matrix which is a Jacobian type  for storing the Jacobian matrix when using computeJointJacobian algorithm
	pinocchio::Data::Matrix6x J(6, model.nv);
	J.setZero(); // intialize the Jacobian matrix to 0;
	/*----------------------------------------------------------------------------------------------*/
	for (int i = 0; ; i++)
	{
		pinocchio::forwardKinematics(model, data, q);//we can get the position and orientation stored in data structure of each joint after using forwardKinematcis algorithm
		//split into two steps to calculate the differece between desired configuration and current configuration
		// 
		// the first step is to obtain the transformation matrxi from the current configuration to the desired configuration
		const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);

		//the second step it to obtain the difference between two configurations by using log6 method;
		err = pinocchio::log6(dMi).toVector();
		/*-----------------------------------------------------------------------------------------------------*/
		if (err.norm() < eps)
		{
			success = true;
			break;
		}
		if (i >= IT_MAX)
		{
			success = false;
			break;
		}
		//updating
		//calculate the jacobian matrix of the end_joint related to the local coordinate
		pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
		pinocchio::Data::Matrix6 JJt;
		JJt.noalias() = J * J.transpose();
		JJt.diagonal().array() += damp;
		v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
		q = pinocchio::integrate(model, q, v * DT);
		if (!(i % 10))
		{
			cout << i << " : error=" << err.transpose() << endl;
		}
	}
	if (success)
	{
		cout << "Convergence achieved !" << endl;
	}
	else
	{
		cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << endl;
	}
	return q;
	//cout << "\nresult is :" << q.transpose() << endl;
	//cout << "\nfinal error is :" << err.transpose() << endl;
}
