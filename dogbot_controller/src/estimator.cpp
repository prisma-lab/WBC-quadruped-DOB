#include "dogbot_model/estimator.h"
#include <random>


ESTIMATOR::ESTIMATOR()
{

}

ESTIMATOR::ESTIMATOR(QUADRUPED &quadruped_)

{
    dogbot = &quadruped_;
   
    yw.resize(4);
    w.resize(4);
    yd.resize(4);
    ygamma.resize(4);
}




void ESTIMATOR::estimate(Eigen::Matrix<double,12,1> &eigenJointVel, Eigen::VectorXd tauj, Eigen::Matrix<double,12,1> Fgrf, std::vector<Eigen::Matrix<double,12,1>>  yd_prev,
                   std::vector<Eigen::Matrix<double,12,1>>  yw_prev, std::vector<Eigen::Matrix<double,12,1>>  w_prev, std::vector<Eigen::Matrix<double,12,1>>  ygamma_prev)
{

 // Get Matrices for the observer
  Eigen::MatrixXd Mj=dogbot->getMassMatrixCOM_joints();
  Eigen::Matrix<double,12,1> q_dot=eigenJointVel;
  Eigen::Matrix<double,12,12> J=dogbot->getJacobianCOM_linear().block(0,6,12,12);
  Eigen::Matrix<double,12,1> Ctq=dogbot->getCtq().block(6,0,12,1);
//Eigen::Matrix<double,12,1> fc=J.transpose()*Fgrf;

// Noise for non ideal situation
 /* const double mean = 0.0;
    const double stddev = 0.1;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);
    const double stddev2 = 0.05;
    std::normal_distribution<double> dist1(mean, stddev2);
   Eigen::VectorXd tau_noise;
   tau_noise.resize(12);
   Eigen::VectorXd fgr_noise;
   fgr_noise=Fgrf;
   for (int i=0;i<12;i++)
   {
    tau_noise[i]=tauj[i]+dist(generator)*tauj[i];
    fgr_noise[i]=fgr_noise[i]+dist1(generator)*fgr_noise[i];
   }*/

// Contact forces
Eigen::Matrix<double,12,1> fc=J.transpose()*Fgrf;
//Terms for the observer
Eigen::MatrixXd p1 = Mj*q_dot;
Eigen::MatrixXd intg=Ctq+tauj+fc;

/*Eigen::Matrix<double,12,1> fc=J.transpose()*fgr_noise;
Eigen::MatrixXd intg=Ctq+tau_noise+fc;*/
 
Eigen::MatrixXd rho=p1;
Eigen::MatrixXd d=intg;

Eigen::VectorXd coeffs=Eigen::VectorXd::Zero(4);
coeffs<<10, 100, 1000, 1;
std::vector<Eigen::Matrix<double,12,12>> k(4);


// Observer
for (int i=0; i<4; i++)
{
   k[i]=coeffs[i]*Eigen::Matrix<double,12,12>::Identity();
}

double T=0.001;
Eigen::Matrix<double,12,12> m=(Eigen::Matrix<double,12,12>::Identity()+k[0]*T).inverse()*k[0];


  yd[0]=yd_prev[0]+(d*T);
  
  w[0] = m*(rho-yw_prev[0]-yd[0]);
  
  yw[0]=yw_prev[0]+w[0]*T;

  for (int i=1; i<3; i++)
  {
    yw[i]=yw_prev[i]+w[i-1]*T;
    w[i] = ((Eigen::Matrix<double,12,12>::Identity()+k[i]*T).inverse()*k[i])*(-ygamma_prev[i]+yw[i]);
    ygamma[i]=ygamma_prev[i]+(w[i]*T);
  }
}


// Compute external forces
Eigen::Matrix<double,12,1> ESTIMATOR::getw3()
{   Eigen::Matrix<double,12,18> J=dogbot->getJacobianCOM_linear();
    Eigen::Matrix<double,12,1> w3=J.block(0,6,12,12).transpose().inverse()*w[2];
    return w3;
}

std::vector<Eigen::Matrix<double,12,1>> ESTIMATOR::getyd()
{ 
    return yd;
}

std::vector<Eigen::Matrix<double,12,1>> ESTIMATOR::getyw()
{   
    return yw;
}

std::vector<Eigen::Matrix<double,12,1>> ESTIMATOR::getw()
{  
    return w;
}

std::vector<Eigen::Matrix<double,12,1>> ESTIMATOR::getygamma()
{   
    return ygamma;
}

