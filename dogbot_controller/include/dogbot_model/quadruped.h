#ifndef QUADRUPEDMODEL
#define QUADRUPEDMODEL

#include <cstdlib>
#include <iostream>
#include <cmath>
// Eigen headers 
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>


// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>
#include "dogbot_model/alglib/optimization.h"


class QUADRUPED
{
  public:
  
  enum SWING_LEGS { L1, L2, L3}; // 4 legs, br-fl, bl-fr
  enum SWING_LEG { BR, FL, BL, FR}; // 4 legs, br-fl, bl-fr

  //robot
   QUADRUPED();
   QUADRUPED(std::string modelFile);
  
  // update the robot state
	void update(Eigen::Matrix4d &eigenWorld_H_base, Eigen::Matrix<double,12,1> &eigenJointPos, Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity);			
  
  // Solve quadratic problem for contact forces
  Eigen::VectorXd qpproblem( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Matrix<double,12,1> &fext);
  Eigen::VectorXd qpproblemtr( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::VectorXd vdotswdes, SWING_LEGS swinglegs, Eigen::Matrix<double,12,1> &fext);
  Eigen::VectorXd qpproblemol( Eigen::Matrix<double,6,1> &Wcom_des, Eigen::Vector3d vdotswdes,  SWING_LEG swingleg, Eigen::Matrix<double,12,1> &fext);

  // get function
	Eigen::VectorXd getBiasMatrix();
	Eigen::VectorXd getGravityMatrix();
	Eigen::MatrixXd getMassMatrix();
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getBiasAcc();
  Eigen::MatrixXd getTransMatrix();
  Eigen::VectorXd getBiasMatrixCOM();
	Eigen::VectorXd getGravityMatrixCOM();
	Eigen::MatrixXd getMassMatrixCOM();
  Eigen::MatrixXd getMassMatrixCOM_com();
  Eigen::MatrixXd getMassMatrixCOM_joints();
  Eigen::MatrixXd getJacobianCOM();
  Eigen::MatrixXd getJacobianCOM_linear();
	Eigen::MatrixXd getBiasAccCOM();
  Eigen::MatrixXd getBiasAccCOM_linear();
  Eigen::MatrixXd getCOMpos();
  Eigen::MatrixXd getCOMvel();
  Eigen::MatrixXd getBRpos();
  Eigen::MatrixXd getBLpos();
  Eigen::MatrixXd getFLpos();
  Eigen::MatrixXd getFRpos();
  Eigen::MatrixXd getBRvel();
  Eigen::MatrixXd getBLvel();
  Eigen::MatrixXd getFLvel();
  Eigen::MatrixXd getFRvel();
  Eigen::MatrixXd getfest();
  Eigen::Matrix<double,3,3> getBRworldtransform();
  Eigen::Matrix<double,3,3> getBLworldtransform();
  Eigen::Matrix<double,3,3> getFLworldtransform();
  Eigen::Matrix<double,3,3> getFRworldtransform();
  Eigen::Matrix<double,3,1> getbrlowerleg();
  double getMass();
  int getDoFsnumber();
  Eigen::MatrixXd getCtq();
  Eigen::MatrixXd getsolution();


  private:
  
  // int for DoFs number
  unsigned int n;
  
  // Total mass of the robot
  double robot_mass;

  // KinDynComputations element
  iDynTree::KinDynComputations kinDynComp;
  
  // world to floating base transformation
  iDynTree::Transform world_H_base;
  
  // Joint position
  iDynTree::VectorDynSize jointPos;
  
  // Floating base velocity
  iDynTree::Twist         baseVel;
  
  // Joint velocity
  iDynTree::VectorDynSize jointVel;
  
  // Gravity acceleration
  iDynTree::Vector3       gravity; 


// Position vector base+joints
  iDynTree::VectorDynSize  qb;

  // Velocity vector base+joints
  iDynTree::VectorDynSize  dqb;

  // Position vector COM+joints
  iDynTree::VectorDynSize  q;

  // Velocity vector COM+joints
  iDynTree::VectorDynSize  dq;

  // Joints limit vector

  iDynTree::VectorDynSize  qmin;
  iDynTree::VectorDynSize  qmax;
  
  // Center of Mass Position

  iDynTree::Vector6 CoM;

  // Center of mass velocity

  iDynTree::Vector6 CoM_vel;
  
  //Mass matrix
  iDynTree::FreeFloatingMassMatrix MassMatrix;
  
  //Bias Matrix
  iDynTree::VectorDynSize Bias;
  
  //Gravity Matrix
  iDynTree::MatrixDynSize GravMatrix;
  
  // Jacobian
  iDynTree::MatrixDynSize Jac;

   // Jacobian derivative
  iDynTree::MatrixDynSize JacDot;
  
  //CoM Jacobian
  iDynTree::MatrixDynSize Jcom;
  
  // Bias acceleration J_dot*q_dot
  iDynTree::MatrixDynSize Jdqd;
  
  // Transformation Matrix
  iDynTree::MatrixDynSize T;
  
  // Transformation matrix time derivative
  iDynTree::MatrixDynSize T_inv_dot;
  
  //Model
  iDynTree::Model model;
  iDynTree::ModelLoader mdlLoader;
  
   //Mass matrix in CoM representation
  iDynTree::FreeFloatingMassMatrix MassMatrixCOM;
  
  
  //Bias Matrix in CoM representation
  iDynTree::VectorDynSize BiasCOM;
  
  //Gravity Matrix in CoM representation
  iDynTree::MatrixDynSize GravMatrixCOM;
  
  // Jacobian in CoM representation
  iDynTree::MatrixDynSize JacCOM;

  //Jacobian in CoM representation (only linear part)
  iDynTree::MatrixDynSize JacCOM_lin;

  // Bias acceleration J_dot*q_dot in CoM representation
  iDynTree::MatrixDynSize JdqdCOM;

   // Bias acceleration J_dot*q_dot in CoM representation
  iDynTree::MatrixDynSize JdqdCOM_lin;
  

   Eigen::VectorXd x_eigen;
   Eigen::Matrix<double,3,1> fest;

   Eigen::Matrix<double,18,1> Ctq;
  void ComputeCtq(Eigen::Matrix<double,18,18> Ctq_pinocchio);
     Eigen::Matrix<double,18,18> Cm;


  //Create the robot
  void createrobot(std::string modelFile);
  
  // Compute the Jacobian
  void  computeJac();

  void  ComputeJaclinear();
  // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
  void computeTransformation(const Eigen::VectorXd &Vel_);
  
  // Compute bias acceleration J_dot*q_dot
  void computeJacDotQDot();
  
  void computeJdqdCOMlinear();
  // Compute Jacobian time derivative
  void computeJacDot(Eigen::Matrix<double, 18,1> Vel_);
  
  //Compute partial derivative
  Eigen::VectorXd  getPartialDerivativeJac(const Eigen::MatrixXd Jacobian, const unsigned int& joint_idx,  const unsigned int& column_idx);
 };



#endif
   


