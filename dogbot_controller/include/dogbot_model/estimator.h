#ifndef ESTIMATORCLASS
#define ESTIMATORCLASS

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

class ESTIMATOR
{

 public:

     ESTIMATOR();
   
     ESTIMATOR(QUADRUPED &quadruped_);
     
     
    void estimate(Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::VectorXd tauj, Eigen::Matrix<double,12,1> Fgrf, std::vector<Eigen::Matrix<double,12,1>>  yd_prev,
                   std::vector<Eigen::Matrix<double,12,1>>  yw_prev, std::vector<Eigen::Matrix<double,12,1>>  w_prev, std::vector<Eigen::Matrix<double,12,1>>  ygamma_prev);

    Eigen::Matrix<double,12,1> getw3();
    std::vector<Eigen::Matrix<double,12,1>> getyd();
    std::vector<Eigen::Matrix<double,12,1>> getyw();
    std::vector<Eigen::Matrix<double,12,1>> getw();
    std::vector<Eigen::Matrix<double,12,1>> getygamma();
 private:

     QUADRUPED*  dogbot;
     std::vector<Eigen::Matrix<double,12,1>>  yd;
     std::vector<Eigen::Matrix<double,12,1>>  yw;
     std::vector<Eigen::Matrix<double,12,1>>  w;
     std::vector<Eigen::Matrix<double,12,1>>  ygamma;
     
};

#endif
