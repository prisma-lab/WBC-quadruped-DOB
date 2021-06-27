#ifndef QUADRUPEDCONTROL
#define QUADRUPEDCONTROL

#include <cstdlib>
#include <iostream>
// Eigen headers 
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include"dogbot_model/quadruped.h"

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

class QUADRUPEDController
{

 public:

     QUADRUPEDController();
   
     QUADRUPEDController(QUADRUPED &quadruped_);
     
     
     Eigen::VectorXd Cntr(iDynTree::Vector6 &CoMPosDes,
                 iDynTree::Vector6 &CoMVelDes,
                 iDynTree::Vector6 &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom,
                 Eigen::Matrix<double,12,1> &fext);

      Eigen::VectorXd CntrBr(iDynTree::Vector6 &CoMPosDes,
                 iDynTree::Vector6 &CoMVelDes,
                 iDynTree::Vector6 &CoMAccDes,
                 Eigen::MatrixXd &Kcom,
                 Eigen::MatrixXd &Dcom, Eigen::VectorXd vdotswdes, QUADRUPED::SWING_LEGS swinglegs, Eigen::Matrix<double,12,1> &fext);       

     Eigen::VectorXd CntrOl(iDynTree::Vector6 &CoMPosDes,
                            iDynTree::Vector6 &CoMVelDes,
                            iDynTree::Vector6 &CoMAccDes,
                            Eigen::MatrixXd &Kcom,
                            Eigen::MatrixXd &Dcom,
                            Eigen::VectorXd vdotswdes,
                            QUADRUPED::SWING_LEG swingleg, Eigen::Matrix<double,12,1> &fext
                            );

 private:

     QUADRUPED*  dogbot;

};

#endif
