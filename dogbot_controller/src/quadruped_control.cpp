#include "dogbot_model/quadruped_control.h"


QUADRUPEDController::QUADRUPEDController()
{

}

QUADRUPEDController::QUADRUPEDController(QUADRUPED &quadruped_)

{
    dogbot = &quadruped_;
  
}


// Control 
Eigen::VectorXd QUADRUPEDController::Cntr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::Matrix<double,12,1> &fext)
{

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();

// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
  Eigen::Matrix<double,12,18> J=dogbot->getJacobianCOM_linear();

// Compute Desired vector
  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);

// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

// Solve quadratic problem
   Eigen::Matrix<double,12,1> fext1=Eigen::Matrix<double,12,1>::Zero();
  tau=dogbot->qpproblem(Wcom_des, fext);
  
  return tau;


}


Eigen::VectorXd QUADRUPEDController::CntrBr(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEGS swinglegs,
                            Eigen::Matrix<double,12,1> &fext)
{
  int swl1, swl2, stl1, stl2;
    switch(swinglegs){
		case QUADRUPED::L1: swl1=0;
		swl2=0 ; 
		stl1=0;
		stl2=0 ; 
		break;
		case QUADRUPED::L2: swl1=0;
		swl2=6 ;
		stl1=3;
		stl2=9 ; 
		 break;
		case QUADRUPED::L3: swl1=3;
		swl2=9;
		stl1=0;
		stl2=6; 
		 break;
	}

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();


// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
   Eigen::Matrix<double,12,18> J=dogbot->getJacobianCOM_linear();
  
   Eigen::Matrix<double,6,6> Jcom;
   Jcom.block(0,0,3,6)=J.block(stl1,0,3,6);
   Jcom.block(3,0,3,6)=J.block(stl2,0,3,6);
   Eigen::Matrix<double,6,1> fext_st;
   fext_st.block(0,0,3,1)=fext.block(stl1,0,3,1);
   fext_st.block(3,0,3,1)=fext.block(stl2,0,3,1);
    Eigen::Matrix<double,6,1> fext_sw;
   fext_sw.block(0,0,3,1)=fext.block(swl1,0,3,1);
   fext_sw.block(3,0,3,1)=fext.block(swl2,0,3,1);

// Desired vector
  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);

// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

// Solve quadratic problem
Eigen::Matrix<double,12,1> fext1= Eigen::Matrix<double,12,1>::Zero();
  tau=dogbot->qpproblemtr(Wcom_des, vdotswdes, swinglegs, fext);
  
  return tau;


}

Eigen::VectorXd QUADRUPEDController::CntrOl(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEG swingleg,
                            Eigen::Matrix<double,12,1> &fext)
{
  int swl1, stl1, stl2, stl3;
    switch(swingleg){
		case QUADRUPED::BR: swl1=0; 
		stl1=3;
		stl2=6 ; 
		stl3=9;
		break;
		case QUADRUPED::FL: swl1=6;
		stl1=0;
		stl2=3 ; 
		stl3=9;
		break;
		case QUADRUPED::BL: swl1=3;
		stl1=0;
		stl2=6; 
		stl3=9;
		break;
		case QUADRUPED::FR: swl1=9;
		stl1=0;
		stl2=3; 
		stl3=6;
		 break;
	}

// Compute deltax, deltav

  Eigen::Matrix<double,6,1> deltax= toEigen(CoMPosDes)-dogbot->getCOMpos();
  Eigen::Matrix<double,6,1> deltav= toEigen(CoMVelDes)-dogbot->getCOMvel();


// Compute desired CoM Wrench
  double mass_robot=dogbot->getMass();

  Eigen::MatrixXd g_acc=Eigen::MatrixXd::Zero(6,1);
  g_acc(2,0)=9.81;
  Eigen::Matrix<double,18,1> deltag=dogbot->getBiasMatrixCOM();
  
  const int n=dogbot->getDoFsnumber();
  Eigen::MatrixXd M_com=dogbot->getMassMatrixCOM_com();
  
  Eigen::Matrix<double,12,18> J=dogbot->getJacobianCOM_linear();
  Eigen::Matrix<double,9,6> Jcom;
   Jcom.block(0,0,3,6)=J.block(stl1,0,3,6);
   Jcom.block(3,0,3,6)=J.block(stl2,0,3,6);
   Jcom.block(6,0,3,6)=J.block(stl3,0,3,6);

  Eigen::Matrix<double,9,1> fext_st;
   fext_st.block(0,0,3,1)=fext.block(stl1,0,3,1);
   fext_st.block(3,0,3,1)=fext.block(stl2,0,3,1);
   fext_st.block(6,0,3,1)=fext.block(stl3,0,3,1);

// Desired vector
  Eigen::Matrix<double,6,1> Wcom_des=Kcom*deltax+Dcom*deltav+mass_robot*g_acc+M_com*toEigen(CoMAccDes);

// Control vector
  Eigen::VectorXd tau= Eigen::VectorXd::Zero(12);

// Solve quadratic problem
  Eigen::Matrix<double,12,1> fext1=Eigen::Matrix<double,12,1>::Zero();
  tau=dogbot->qpproblemol(Wcom_des, vdotswdes, swingleg, fext);
  
  return tau;


}
