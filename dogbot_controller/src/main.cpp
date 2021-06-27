#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
// Eigen headers 
#include <Eigen/Core>
#include "dogbot_model/quadruped.h"
#include "dogbot_model/quadruped_control.h"
#include "dogbot_model/estimator.h"
// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>
// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/GetLinkProperties.h"
#include "gazebo_msgs/SetLinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelConfiguration.h"
#include "gazebo_msgs/SetModelState.h"
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <towr/initialization/gait_generator.h>
#include "gazebo_msgs/ContactsState.h"
#include <chrono>  
#include <random>


// Global variables
  // Joint variables
     Eigen::Matrix<double,12,1> jnt_pos, jnt_vel;
     Eigen::Matrix<double,6,1>  base_pos,  base_vel;
     Eigen::Matrix4d world_H_base;
  // Boolean variables
     bool joint_state_available = false, base_state_available = false, contact_br= true, contact_fl= true, contact_fr= true, contact_bl= true;
     Eigen::Vector3d gravity;
  // Subscribers
     ros::Subscriber _eebl_sub , _eefl_sub , _eebr_sub, _eefr_sub, _distxbr_sub, _distybr_sub, _distxbl_sub, _distybl_sub, _distxfl_sub, _distyfl_sub, _distxfr_sub, _distyfr_sub;
     double distx_br, disty_br, distx_bl, disty_bl, distx_fl, disty_fl, distx_fr, disty_fr;
  // Publisher
     ros::Publisher joint_effort_pub;
     gazebo::transport::PublisherPtr pub;
     gazebo::msgs::WorldControl stepper;  
     std_msgs::Float64MultiArray tau1_msg;
  // Variables for the control algorithm
     QUADRUPEDController* controller_ ;
     ESTIMATOR* estimator_;
     ros::Time begin;
     QUADRUPED* doggo;
     Eigen::MatrixXd Kcom;
     Eigen::MatrixXd Dcom;
     Eigen::VectorXd tau;     
     Eigen::Matrix<double,3,1>  force_br, force_bl, force_fl, force_fr;
     std::vector<Eigen::Matrix<double,12,1>>  yd_prev,  yw_prev, w_prev, ygamma_prev;
     std::vector<Eigen::Matrix<double,6,1>>  yd_prev_sem,  yw_prev_sem, w_prev_sem, ygamma_prev_sem;
     Eigen::Matrix<double,12,1> Fgrf;
     ignition::transport::Node node_ign;
     ignition::msgs::Marker markerMsg;
  // Towr variables
     towr::SplineHolder solution;
     towr::NlpFormulation formulation;
     int flag;

    std::ofstream com_file("com_file.txt");
    std::ofstream com_vel_file("com_vel_file.txt");
    std::ofstream com_des_file("com_des_file.txt");
    std::ofstream com_vel_des_file("com_vel_des_file.txt");
    std::ofstream pbr_des_file("pbr_des_file.txt");
    std::ofstream pbr_file("pbr_file.txt");
    std::ofstream pbl_des_file("pbl_des_file.txt");
    std::ofstream pbl_file("pbl_file.txt");
    std::ofstream pfl_des_file("pfl_des_file.txt");
    std::ofstream pfl_file("pfl_file.txt");
    std::ofstream pfr_des_file("pfr_des_file.txt");
    std::ofstream pfr_file("pfr_file.txt");
    std::ofstream fgr_des_file("fgr_des_file.txt");
    std::ofstream fgr_file("fgr_file.txt");
    std::ofstream fest_file("fest_file.txt");
    std::ofstream fext_file("fext_file.txt");
    std::ofstream tau_file("tau_file.txt");
    std::ofstream joint_file("joint_file.txt");
    std::ofstream qb_file("qb_file.txt");    
    std::ofstream ddqopt_file("ddqopt_file.txt");


// Get joints position and velocity
void jointStateCallback(const sensor_msgs::JointState & msg)
{
    joint_state_available = true;
 
     jnt_pos(0,0)=msg.position[11];
     jnt_pos(1,0)=msg.position[8];
     jnt_pos(2,0)=msg.position[2];
     jnt_pos(3,0)=msg.position[5];
     jnt_pos(4,0)=msg.position[4];
     jnt_pos(5,0)=msg.position[3];
     jnt_pos(6,0)=msg.position[1];
     jnt_pos(7,0)=msg.position[0];
     jnt_pos(8,0)=msg.position[7];
     jnt_pos(9,0)=msg.position[6];
     jnt_pos(10,0)=msg.position[10];
     jnt_pos(11,0)=msg.position[9];

     jnt_vel(0,0)=msg.velocity[11];
     jnt_vel(1,0)=msg.velocity[8];
     jnt_vel(2,0)=msg.velocity[2];
     jnt_vel(3,0)=msg.velocity[5];
     jnt_vel(4,0)=msg.velocity[4];
     jnt_vel(5,0)=msg.velocity[3];
     jnt_vel(6,0)=msg.velocity[1];
     jnt_vel(7,0)=msg.velocity[0];
     jnt_vel(8,0)=msg.velocity[7];
     jnt_vel(9,0)=msg.velocity[6];
     jnt_vel(10,0)=msg.velocity[10];
     jnt_vel(11,0)=msg.velocity[9];

  double t = (ros::Time::now()).toSec();
  std::cout<<"tempo_joint"<<t<<std::endl;
}


// Get base position and velocity
void modelStateCallback(const gazebo_msgs::ModelStates & msg)
{
    base_state_available = true;
    world_H_base.setIdentity();

      //quaternion
         tf::Quaternion q(msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z,  msg.pose[1].orientation.w);
         q.normalize();
         Eigen::Matrix<double,3,3> rot;
         tf::matrixTFToEigen(tf::Matrix3x3(q),rot);
     
      //Roll, pitch, yaw
         double roll, pitch, yaw;
         tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
      //Set base pos (position and orientation)
         base_pos<<msg.pose[1].position.x, msg.pose[1].position.y, msg.pose[1].position.z, roll, pitch, yaw;
      //Set transformation matrix
         world_H_base.block(0,0,3,3)= rot;
         world_H_base.block(0,3,3,1)= base_pos.block(0,0,3,1);
  
      //Set base vel
         base_vel<<msg.twist[1].linear.x, msg.twist[1].linear.y, msg.twist[1].linear.z, msg.twist[1].angular.x, msg.twist[1].angular.y, msg.twist[1].angular.z;
}

// Feet's state
void eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

	if(eebr->states.empty()){ 
    contact_br= false;
	}
	else{
		contact_br= true;
    force_br<<eebr->states[0].total_wrench.force.x, eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z;
	}
}

void eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

	if(eefl->states.empty()){ 
    contact_fl= false;
	}
	else{
		contact_fl= true;
    force_fl<<eefl->states[0].total_wrench.force.x, eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z;

	}
}

void eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){

	if(eebl->states.empty()){ 
    contact_bl= false;
	}
	else{
		contact_bl= true;
    force_bl<<eebl->states[0].total_wrench.force.x, eebl->states[0].total_wrench.force.y, eebl->states[0].total_wrench.force.z;

	}
}

void eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){

	if(eefr->states.empty()){ 
    contact_fr= false;
	}
	else{
		contact_fr= true;
    force_fr<<eefr->states[0].total_wrench.force.x, eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z;

	}
}

void distxbr_cb(std_msgs::Float64ConstPtr dist_x){

	distx_br=dist_x->data;
}

void distybr_cb(std_msgs::Float64ConstPtr dist_y){

	disty_br=dist_y->data;
}

void distxbl_cb(std_msgs::Float64ConstPtr dist_x){

	distx_bl=dist_x->data;
}

void distybl_cb(std_msgs::Float64ConstPtr dist_y){

	disty_bl=dist_y->data;
}

void distxfl_cb(std_msgs::Float64ConstPtr dist_x){

	distx_fl=dist_x->data;
}

void distyfl_cb(std_msgs::Float64ConstPtr dist_y){

	disty_fl=dist_y->data;
}
void distxfr_cb(std_msgs::Float64ConstPtr dist_x){

	distx_fr=dist_x->data;
}

void distyfr_cb(std_msgs::Float64ConstPtr dist_y){

	disty_fr=dist_y->data;
}


// Stand Phase Function
void stand_phase( ros::Rate loop_rate, double duration )
{ while((ros::Time::now()-begin).toSec() < duration)
   if (joint_state_available && base_state_available)
      {  
        
        // Update robot
           doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
           double t = (ros::Time::now()-begin).toSec();

        // Set desired vectors
           iDynTree::Vector6 composdes, comveldes, comaccdes;
 
           toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
           toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
           toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

        // Get current ground reaction forces
           Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
           Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
           Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
           Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
           Fgrf<< Tbr*force_br, Tbl*force_bl,Tfl*force_fl,Tfr*force_fr;

        // Observer
           estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);
           Eigen::Matrix<double,12,1> fext =estimator_->getw3();
           yd_prev =estimator_->getyd();
           yw_prev =estimator_->getyw();
           w_prev =estimator_->getw();
           ygamma_prev =estimator_->getygamma();

        // Compute torques
           tau = controller_->Cntr(composdes, comveldes, comaccdes,
                                    Kcom, Dcom,fext);
                                 
        // Set command message
           tau1_msg.data.clear();
           std::vector<double> ta(12,0.0);

        // torques in right order
           ta[11]=tau(7);
           ta[10]=tau(6);
           ta[9]=tau(2);
           ta[8]=tau(5);
           ta[7]=tau(4);
           ta[6]=tau(3);
           ta[5]=tau(9);
           ta[4]=tau(8);
           ta[3]=tau(1);
           ta[2]=tau(11);
           ta[1]=tau(10);
           ta[0]=tau(0);

        // Fill Command message
           for(int i=0; i<12; i++)
           {
             tau1_msg.data.push_back(ta[i]);
           }

        //Sending command
           joint_effort_pub.publish(tau1_msg);
        
    
      
    
    ////////
    // Save data

    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution_qp=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution_qp.block(18,0,12,1);

    fgr_des_file<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<" "<<fgr_des(6)<<" "<<fgr_des(7)<<" "<<fgr_des(8)<<" "<<fgr_des(9)<<" "<<fgr_des(10)<<" "<<fgr_des(11)<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
     fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
  
    fest_file.flush();

    fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

    joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();

    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution_qp.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

   
    // One step in gazebo world ( to use if minqp problem takes too long for control loop)
        pub->Publish(stepper);
        ros::spinOnce();
        loop_rate.sleep();
      }
}


// Swing phase function (back right leg and front left leg)
void swing_phase( ros::Rate loop_rate, double duration , double duration_prev)
{while ((ros::Time::now()-begin).toSec() < duration && flag==0 )
    {   
      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);
             
        // Time
        double t = (ros::Time::now()-begin).toSec();

        // Set desired vectors for CoM
         iDynTree::Vector6 composdes, comveldes, comaccdes;
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

        // Set desired vectors for swing feet
        Eigen::Matrix<double,6,1> accd;
         accd<< solution.ee_motion_.at(1)->GetPoint(t).a(),
                solution.ee_motion_.at(2)->GetPoint(t).a();

         Eigen::Matrix<double,6,1> posdelta;
         posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos(),
                    solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos();

         Eigen::Matrix<double,6,1> veldelta;
         veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel(),
                    solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel();
        
         Eigen::MatrixXd Kp;
         Kp=250*Eigen::MatrixXd::Identity(6,6);
         Eigen::MatrixXd Kd;
         Kd=50*Eigen::MatrixXd::Identity(6,6);

         Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        // Compute ground reaction forces
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
         Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
         Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
         Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
       
         Fgrf<< Eigen::Matrix<double,3,1>::Zero(), Tbl*force_bl, Eigen::Matrix<double,3,1>::Zero(), Tfr*force_fr;
        
        // Observer  
         estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);

         Eigen::Matrix<double,12,1> fext =estimator_->getw3();
         yd_prev =estimator_->getyd();
         yw_prev =estimator_->getyw();
         w_prev =estimator_->getw();
         ygamma_prev =estimator_->getygamma();

        // Compute torque
         tau = controller_->CntrBr(composdes, comveldes, comaccdes,
                                   Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L2, fext);
                                 
        // Set command message
         tau1_msg.data.clear();
         std::vector<double> ta(12,0.0);
        // torques in right order
         ta[11]=tau(7);
         ta[10]=tau(6);
         ta[9]=tau(2);
         ta[8]=tau(5);
         ta[7]=tau(4);
         ta[6]=tau(3);
         ta[5]=tau(9);
         ta[4]=tau(8);
         ta[3]=tau(1);
         ta[2]=tau(11);
         ta[1]=tau(10);
         ta[0]=tau(0);

        // Fill Command message
         for(int i=0; i<12; i++)
         {
   
          tau1_msg.data.push_back(ta[i]);
         }

        //Sending command
         joint_effort_pub.publish(tau1_msg);

    
      
     ////////

     // Save data

    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution.block(18,0,6,1);

    fgr_des_file<<0<<" "<<0<<" "<<0<<" "<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<0<<" "<<0<<" "<<0<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
    // Eigen::Matrix<double,12,1> w3 =estimator_->getw3();
         
   fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
  fest_file.flush();

    fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

     joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();

    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

    ////////////////////////////////////
    // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    pub->Publish(stepper);
        ros::spinOnce();
       if(contact_br==false && contact_fl==true && t>duration-0.1)
      {flag=1;}
      else if(contact_br==true && contact_fl==false && t>duration-0.1)
      {flag=2;}
      else if(contact_br==true && contact_fl==true && t>duration-0.1)
      {flag=3;}
    loop_rate.sleep();
        
      
    }
    }
}

// Swing phase function (back left leg and front right leg)
void swing_phase2( ros::Rate loop_rate, double duration , double duration_prev)
 {while ((ros::Time::now()-begin).toSec() < duration && flag==0 )
    { 
     if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
        double t = (ros::Time::now()-begin).toSec();

        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;

         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
         
        // Set desired vectors for feet
         Eigen::Matrix<double,6,1> accd;
         accd<< solution.ee_motion_.at(0)->GetPoint(t).a(),
                solution.ee_motion_.at(3)->GetPoint(t).a();

         Eigen::Matrix<double,6,1> posdelta;
         posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos(),
                    solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

         Eigen::Matrix<double,6,1> veldelta;
         veldelta<< solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel(),
                    solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
        
         Eigen::MatrixXd Kp;
         Kp=250*Eigen::MatrixXd::Identity(6,6);
         Eigen::MatrixXd Kd;
         Kd=50*Eigen::MatrixXd::Identity(6,6);

         Eigen::Matrix<double,6,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        // Compute ground reaction forces
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
         Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
         Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
         Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();
         Fgrf<<Tbr*force_br, Eigen::Matrix<double,3,1>::Zero(),  Tfl*force_fl, Eigen::Matrix<double,3,1>::Zero();
        
        // Observer
         estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);
         Eigen::Matrix<double,12,1> fext =estimator_->getw3();
         yd_prev =estimator_->getyd();
         yw_prev =estimator_->getyw();
         w_prev =estimator_->getw();
         ygamma_prev =estimator_->getygamma();

        // Compute torque
         tau = controller_->CntrBr(composdes, comveldes, comaccdes,
                                   Kcom, Dcom, accdes, QUADRUPED::SWING_LEGS::L3,fext);
                                 
        // Set command message
         tau1_msg.data.clear();
         std::vector<double> ta(12,0.0);

        // torques in right order
         ta[11]=tau(7);
         ta[10]=tau(6);
         ta[9]=tau(2);
         ta[8]=tau(5);
         ta[7]=tau(4);
         ta[6]=tau(3);
         ta[5]=tau(9);
         ta[4]=tau(8);
         ta[3]=tau(1);
         ta[2]=tau(11);
         ta[1]=tau(10);
         ta[0]=tau(0);

        // Fill Command message
         for(int i=0; i<12; i++)
         {
          tau1_msg.data.push_back(ta[i]);
         }

        //Sending command
         joint_effort_pub.publish(tau1_msg);
    
        ///////////////////
    // save data
    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution.block(18,0,6,1);

    fgr_des_file<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<0<<" "<<0<<" "<<0<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<" "<<0<<" "<<0<<" "<<0<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
    w3=J2.block(0,6,12,12).transpose()*w3;      
    fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
       fest_file.flush();

    fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

    joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();

    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

  
    // One step in gazebo world ( to use if minqp problem takes too long for control loop)
      pub->Publish(stepper);
      
      ros::spinOnce();
      if(contact_fr==false && contact_bl==true && t>duration-0.1)
      {flag=1;}
      else if(contact_fr==true && contact_bl==false && t>duration-0.1)
      {flag=2;}
      else if(contact_fr==true && contact_bl==true && t>duration-0.1)
      {flag=2;}
        loop_rate.sleep();
         }
    }
    }


// Swing phase function (back right leg)
  void swing_phasebr( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin).toSec() < duration &  flag == 0)
    { 

      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
         double t = (ros::Time::now()-begin).toSec();

        // Set desired vectors for CoM
         iDynTree::Vector6 composdes, comveldes, comaccdes;

         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

        // Set desired vectors for foot
         Eigen::Matrix<double,3,1> accd;
         accd<< solution.ee_motion_.at(1)->GetPoint(t).a();

         Eigen::Matrix<double,3,1> posdelta;
         posdelta<< solution.ee_motion_.at(1)->GetPoint(t).p()-doggo->getBRpos();

         Eigen::Matrix<double,3,1> veldelta;
         veldelta<< solution.ee_motion_.at(1)->GetPoint(t).v()-doggo->getBRvel();
        
         Eigen::MatrixXd Kp;
         Kp=250*Eigen::MatrixXd::Identity(3,3);
         Eigen::MatrixXd Kd;
         Kd=50*Eigen::MatrixXd::Identity(3,3);

         Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        // Compute ground reaction forces
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
         Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
         Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
         Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();

         Fgrf<< Eigen::Matrix<double,3,1>::Zero(),Tbl*force_bl,  Tfl*force_fl, Tfr*force_fr;
        
        // Observer
         estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);
        
         Eigen::Matrix<double,12,1> fext =estimator_->getw3();
         yd_prev =estimator_->getyd();
         yw_prev =estimator_->getyw();
         w_prev =estimator_->getw();
         ygamma_prev =estimator_->getygamma();


        // Compute torques
         tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                   Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::BR,fext); 
       

        // Set command message
         tau1_msg.data.clear();
         std::vector<double> ta(12,0.0);

        // torques in right order
         ta[11]=tau(7);
         ta[10]=tau(6);
         ta[9]=tau(2);
         ta[8]=tau(5);
         ta[7]=tau(4);
         ta[6]=tau(3);
         ta[5]=tau(9);
         ta[4]=tau(8);
         ta[3]=tau(1);
         ta[2]=tau(11);
         ta[1]=tau(10);
         ta[0]=tau(0);


        // Fill Command message
         for(int i=0; i<12; i++)
        {
         tau1_msg.data.push_back(ta[i]);
        }

        //Sending command
         joint_effort_pub.publish(tau1_msg);
     

    ////////
    // save data

    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution.block(18,0,9,1);

    fgr_des_file<<0<<" "<<0<<" "<<0<<" "<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<" "<<fgr_des(6)<<" "<<fgr_des(7)<<" "<<fgr_des(8)<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
    fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
     fest_file.flush();

   fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

    joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();

    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

    /////////////////////////
     // One step in gazebo world ( to use if minqp problem takes too long for control loop)
    pub->Publish(stepper);
        ros::spinOnce();
        loop_rate.sleep();

      if(contact_br==true && t>duration-0.1)
      {flag=1;}
        }
     }
   }


 // swing phase function (front left leg)
  void swing_phasefl( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin).toSec() < duration &  flag == 0)
    { 

      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
         double t = (ros::Time::now()-begin).toSec();
      
        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;

         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
       
         Eigen::Matrix<double,3,1> accd;
         accd<< solution.ee_motion_.at(2)->GetPoint(t).a();

         Eigen::Matrix<double,3,1> posdelta;
         posdelta<< solution.ee_motion_.at(2)->GetPoint(t).p()-doggo->getFLpos();

         Eigen::Matrix<double,3,1> veldelta;
         veldelta<< solution.ee_motion_.at(2)->GetPoint(t).v()-doggo->getFLvel();
        
         Eigen::MatrixXd Kp;
         Kp=250*Eigen::MatrixXd::Identity(3,3);
         Eigen::MatrixXd Kd;
         Kd=50*Eigen::MatrixXd::Identity(3,3);

         Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        // Compute ground reaction forces
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
         Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
         Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
         Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();

         Fgrf<< Tbr*force_br, Tbl*force_bl, Eigen::Matrix<double,3,1>::Zero(), Tfr*force_fr;

        // Observer
         estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);

         Eigen::Matrix<double,12,1> fext =estimator_->getw3();
         yd_prev =estimator_->getyd();
         yw_prev =estimator_->getyw();
         w_prev =estimator_->getw();
         ygamma_prev =estimator_->getygamma();
        
        // Compute torques
         tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                   Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::FL,fext); 
       

        // Set command message
         tau1_msg.data.clear();
         std::vector<double> ta(12,0.0);

        // torques in right order
         ta[11]=tau(7);
         ta[10]=tau(6);
         ta[9]=tau(2);
         ta[8]=tau(5);
         ta[7]=tau(4);
         ta[6]=tau(3);
         ta[5]=tau(9);
         ta[4]=tau(8);
         ta[3]=tau(1);
         ta[2]=tau(11);
         ta[1]=tau(10);
         ta[0]=tau(0);

        // Fill Command message
         for(int i=0; i<12; i++)
          {
           tau1_msg.data.push_back(ta[i]);
          }

        // Sending command
          joint_effort_pub.publish(tau1_msg);

    
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
     
          ////////
    // save data
    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution.block(18,0,9,1);

    fgr_des_file<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<" "<<0<<" "<<0<<" "<<0<<" "<<fgr_des(6)<<" "<<fgr_des(7)<<" "<<fgr_des(8)<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
    fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
    fest_file.flush();

    fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

   joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();


    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

    /////////////////////////

        
      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
        pub->Publish(stepper);
        ros::spinOnce();
        loop_rate.sleep();
        
        if(contact_fl==true && t>duration-0.1)
        {flag=1;}
      
        }
     }
    }

// Swing Phase function (back left leg)
  void swing_phasebl( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin).toSec() < duration &  flag == 0)
    { 

      if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
         double t = (ros::Time::now()-begin).toSec();

        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
     
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();

         Eigen::Matrix<double,3,1> accd;
         accd<< solution.ee_motion_.at(0)->GetPoint(t).a();

         Eigen::Matrix<double,3,1> posdelta;
         posdelta<< solution.ee_motion_.at(0)->GetPoint(t).p()-doggo->getBLpos();

         Eigen::Matrix<double,3,1> veldelta;
         veldelta<< solution.ee_motion_.at(0)->GetPoint(t).v()-doggo->getBLvel();
        
         Eigen::MatrixXd Kp;
         Kp=250*Eigen::MatrixXd::Identity(3,3);
         Eigen::MatrixXd Kd;
         Kd=50*Eigen::MatrixXd::Identity(3,3);

         Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        // Compute ground reaction forces
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
         Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
         Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
         Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();

         Fgrf<<Tbr*force_br, Eigen::Matrix<double,3,1>::Zero(),  Tfl*force_fl, Tfr*force_fr;

        // Observer
          estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);
        
          Eigen::Matrix<double,12,1> fext =estimator_->getw3();
          yd_prev =estimator_->getyd();
          yw_prev =estimator_->getyw();
          w_prev =estimator_->getw();
          ygamma_prev =estimator_->getygamma();

        // Compute torques
          tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                    Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::BL,fext); 

        // Set command message
         tau1_msg.data.clear();
         std::vector<double> ta(12,0.0);

        // torques in right order
         ta[11]=tau(7);
         ta[10]=tau(6);
         ta[9]=tau(2);
         ta[8]=tau(5);
         ta[7]=tau(4);
         ta[6]=tau(3);
         ta[5]=tau(9);
         ta[4]=tau(8);
         ta[3]=tau(1);
         ta[2]=tau(11);
         ta[1]=tau(10);
         ta[0]=tau(0);

        // Fill Command message
         for(int i=0; i<12; i++)
          {
           tau1_msg.data.push_back(ta[i]);
          }

        //Sending command
          joint_effort_pub.publish(tau1_msg);

             ////////

    // save data
    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution.block(18,0,9,1);

    fgr_des_file<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<0<<" "<<0<<" "<<0<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<" "<<fgr_des(6)<<" "<<fgr_des(7)<<" "<<fgr_des(8)<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
     fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
    fest_file.flush();

    fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

    joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();


    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

    /////////////////////////

      // One step in gazebo world ( to use if minqp problem takes too long for control loop)
        pub->Publish(stepper);
        ros::spinOnce();
        loop_rate.sleep();
        if(contact_bl==true && t>duration-0.1)
        {flag=1;}    
      }
    }
  }

// Swing Phase Function (front right leg)
  void swing_phasefr( ros::Rate loop_rate, double duration , double duration_prev)
  { while ((ros::Time::now()-begin).toSec() < duration &  flag==0)
    { if (joint_state_available && base_state_available)
      {  
        // Update robot
         doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

        // Time
        double t = (ros::Time::now()-begin).toSec();

        // Set desired vectors
         iDynTree::Vector6 composdes, comveldes, comaccdes;
       
         toEigen(composdes)<<solution.base_linear_->GetPoint(t).p(), solution.base_angular_->GetPoint(t).p();
         toEigen(comveldes)<<solution.base_linear_->GetPoint(t).v(), solution.base_angular_->GetPoint(t).v();
         toEigen(comaccdes) <<solution.base_linear_->GetPoint(t).a(), solution.base_angular_->GetPoint(t).a();
       
         Eigen::Matrix<double,3,1> accd;
         accd<< solution.ee_motion_.at(3)->GetPoint(t).a();

         Eigen::Matrix<double,3,1> posdelta;
         posdelta<< solution.ee_motion_.at(3)->GetPoint(t).p()-doggo->getFRpos();

         Eigen::Matrix<double,3,1> veldelta;
         veldelta<< solution.ee_motion_.at(3)->GetPoint(t).v()-doggo->getFRvel();
        
         Eigen::MatrixXd Kp;
         Kp=250*Eigen::MatrixXd::Identity(3,3);
         Eigen::MatrixXd Kd;
         Kd=50*Eigen::MatrixXd::Identity(3,3);

         Eigen::Matrix<double,3,1> accdes=accd+Kd*veldelta+Kp*posdelta;

        // Compute ground reaction forces
         Eigen::Matrix<double,3,3> Tbr= doggo->getBRworldtransform();
         Eigen::Matrix<double,3,3> Tbl= doggo->getBLworldtransform();
         Eigen::Matrix<double,3,3> Tfl= doggo->getFLworldtransform();
         Eigen::Matrix<double,3,3> Tfr=doggo->getFRworldtransform();

         Fgrf<<Tbr*force_br, Tbl*force_bl,  Tfl*force_fl, Eigen::Matrix<double,3,1>::Zero();
        
        // Observer
         estimator_->estimate(jnt_vel, tau, Fgrf, yd_prev, yw_prev, w_prev,  ygamma_prev);

         Eigen::Matrix<double,12,1> fext =estimator_->getw3();
         yd_prev =estimator_->getyd();
         yw_prev =estimator_->getyw();
         w_prev =estimator_->getw();
         ygamma_prev =estimator_->getygamma();
    
        // Compute torques
        tau = controller_->CntrOl(composdes, comveldes, comaccdes,
                                  Kcom, Dcom, accdes, QUADRUPED::SWING_LEG::FR,fext); 
  
        // Set command message
         tau1_msg.data.clear();
         std::vector<double> ta(12,0.0);

        // torques in right order
         ta[11]=tau(7);
         ta[10]=tau(6);
         ta[9]=tau(2);
         ta[8]=tau(5);
         ta[7]=tau(4);
         ta[6]=tau(3);
         ta[5]=tau(9);
         ta[4]=tau(8);
         ta[3]=tau(1);
         ta[2]=tau(11);
         ta[1]=tau(10);
         ta[0]=tau(0);


        // Fill Command message
         for(int i=0; i<12; i++)
         {
          tau1_msg.data.push_back(ta[i]);
         }

       //Sending command
        joint_effort_pub.publish(tau1_msg);

    
             ////////
    // Save data
    /*Eigen::MatrixXd com= doggo->getCOMpos();
    Eigen::MatrixXd com_vel= doggo->getCOMvel();

    com_file<<com(0)<<" "<<com(1)<<" "<<com(2)<<" "<<com(3)<<" "<<com(4)<<" "<<com(5)<<"\n";
    com_vel_file<<com_vel(0)<<" "<<com_vel(1)<<" "<<com_vel(2)<<" "<<com_vel(3)<<" "<<com_vel(4)<<" "<<com_vel(5)<<"\n";
    com_des_file<<composdes(0)<<" "<<composdes(1)<<" "<<composdes(2)<<" "<<composdes(3)<<" "<<composdes(4)<<" "<<composdes(5)<<"\n";
    com_vel_des_file<<comveldes(0)<<" "<<comveldes(1)<<" "<<comveldes(2)<<" "<<comveldes(3)<<" "<<comveldes(4)<<" "<<comveldes(5)<<"\n";
    com_file.flush();
    com_vel_file.flush();
    com_des_file.flush();
    com_vel_des_file.flush();

    Eigen::MatrixXd pbr=doggo->getBRpos();
    Eigen::MatrixXd pbr_des=solution.ee_motion_.at(1)->GetPoint(t).p();

    pbr_des_file<<pbr_des(0)<<" "<<pbr_des(1)<<" "<<pbr_des(2)<<"\n";
    pbr_file<<pbr(0)<<" "<<pbr(1)<<" "<<pbr(2)<<"\n";
    pbr_des_file.flush();
    pbr_file.flush();

    Eigen::MatrixXd pbl=doggo->getBLpos();
    Eigen::MatrixXd pbl_des=solution.ee_motion_.at(0)->GetPoint(t).p();

    pbl_des_file<<pbl_des(0)<<" "<<pbl_des(1)<<" "<<pbl_des(2)<<"\n";
    pbl_file<<pbl(0)<<" "<<pbl(1)<<" "<<pbl(2)<<"\n";
    pbl_des_file.flush();
    pbl_file.flush();

    Eigen::MatrixXd pfl=doggo->getFLpos();
    Eigen::MatrixXd pfl_des=solution.ee_motion_.at(2)->GetPoint(t).p();

    pfl_des_file<<pfl_des(0)<<" "<<pfl_des(1)<<" "<<pfl_des(2)<<"\n";
    pfl_file<<pfl(0)<<" "<<pfl(1)<<" "<<pfl(2)<<"\n";
    pfl_des_file.flush();
    pfl_file.flush();

    Eigen::MatrixXd pfr=doggo->getFRpos();
    Eigen::MatrixXd pfr_des=solution.ee_motion_.at(3)->GetPoint(t).p();

    pfr_des_file<<pfr_des(0)<<" "<<pfr_des(1)<<" "<<pfr_des(2)<<"\n";
    pfr_file<<pfr(0)<<" "<<pfr(1)<<" "<<pfr(2)<<"\n";
    pfr_des_file.flush();
    pfr_file.flush();


    Eigen::MatrixXd solution=doggo->getsolution();
    Eigen::MatrixXd fgr_des=solution.block(18,0,9,1);

    fgr_des_file<<fgr_des(0)<<" "<<fgr_des(1)<<" "<<fgr_des(2)<<" "<<fgr_des(3)<<" "<<fgr_des(4)<<" "<<fgr_des(5)<<" "<<fgr_des(6)<<" "<<fgr_des(7)<<" "<<fgr_des(8)<<" "<<0<<" "<<0<<" "<<0<<"\n";
    fgr_des_file.flush();

    fgr_file<<Fgrf(0)<<" "<<Fgrf(1)<<" "<<Fgrf(2)<<" "<<Fgrf(3)<<" "<<Fgrf(4)<<" "<<Fgrf(5)<<" "<<Fgrf(6)<<" "<<Fgrf(7)<<" "<<Fgrf(8)<<" "<<Fgrf(9)<<" "<<Fgrf(10)<<" "<<Fgrf(11)<<"\n";
    fgr_file.flush();

    Eigen::MatrixXd fest=doggo->getfest();
      fest_file<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(0)<<" "<<fext(1)<<" "<<fext(2)<<" "<<fext(6)<<" "<<fext(7)<<" "<<fext(8)<<" "<<fext(9)<<" "<<fext(10)<<" "<<fext(11)<<" "<<"\n";
    fest_file.flush();

   fext_file<<distx_br<<" "<<disty_br<<" "<<distx_bl<<" "<<disty_bl<<" "<<distx_fl<<" "<<disty_fl<<" "<<distx_fr<<" "<<disty_fr<<"\n";
    fext_file.flush();

    tau_file<<ta[11]<<" "<<ta[10]<<" "<<ta[9]<<" "<<ta[8]<<" "<<ta[7]<<" "<<ta[6]<<" "<<ta[5]<<" "<<ta[4]<<" "<<ta[3]<<" "<<ta[2]<<" "<<ta[1]<<" "<<ta[0]<<"\n";
    tau_file.flush();

    joint_file<<jnt_vel(7,0)<<" "<<jnt_vel(6,0)<<" "<<jnt_vel(2,0)<<" "<<jnt_vel(5,0)<<" "<<jnt_vel(4,0)<<" "<<jnt_vel(3,0)<<" "<<jnt_vel(9,0)<<" "<<jnt_vel(8,0)<<" "<<jnt_vel(1,0)<<" "<<jnt_vel(11,0)<<" "<<jnt_vel(10,0)<<" "<<jnt_vel(0,0)<<"\n";
    joint_file.flush();

    qb_file<<base_pos(0)<<" "<<base_pos(1)<<" "<<base_pos(2)<<" "<<base_pos(3)<<" "<<base_pos(4)<<" "<<base_pos(5)<<"\n";
     qb_file.flush();

    Eigen::MatrixXd ddq_des=solution.block(0,0,18,1);

    ddqopt_file<<ddq_des(0)<<" "<<ddq_des(1)<<" "<<ddq_des(2)<<" "<<ddq_des(3)<<" "<<ddq_des(4)<<" "<<ddq_des(5)<<" "<<ddq_des(6)<<" "<<ddq_des(7)<<" "<<ddq_des(8)<<" "<<ddq_des(9)<<" "<<ddq_des(10)<<" "<<ddq_des(11)
              <<" "<<ddq_des(12)<<" "<<ddq_des(13)<<" "<<ddq_des(14)<<" "<<ddq_des(15)<<" "<<ddq_des(16)<<" "<<ddq_des(17)<<"\n";
    ddqopt_file.flush();*/

    /////////////////////////

     // One step in gazebo world ( to use if minqp problem takes too long for control loop)
       pub->Publish(stepper);
       ros::spinOnce();
       loop_rate.sleep();
      if(contact_fr==true && t>duration-0.1)
      {flag=1;}
      
       }
    }
  }


// Replan trajectory
 towr::NlpFormulation computetrajecotry(int gait_flag)
 {   towr::NlpFormulation formulation_;
    // terrain
  formulation_.terrain_ = std::make_shared<towr::FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the robot
  formulation_.model_ = towr::RobotModel(towr::RobotModel::Dogbot);

  formulation_.initial_base_.lin.at(towr::kPos) << doggo->getCOMpos().block(0,0,3,1);
  formulation_.initial_base_.ang.at(towr::kPos) << doggo->getCOMpos().block(3,0,3,1);
  formulation_.initial_base_.lin.at(towr::kVel) << doggo->getCOMvel().block(0,0,3,1);
  formulation_.initial_base_.ang.at(towr::kVel) << doggo->getCOMvel().block(3,0,3,1);

  // Trajectory
   if((ros::Time::now()).toSec()>7 && (ros::Time::now()).toSec()<17)
    {formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
     formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]-0.025;}
   else if((ros::Time::now()).toSec()<7){
     formulation_.final_base_.lin.at(towr::kPos) << 0.0, formulation_.initial_base_.lin.at(towr::kPos)[1]-0.05, 0.40229;
     formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, 0.0;
   }
   else if((ros::Time::now()).toSec()>17  && (ros::Time::now()).toSec()<24 )
    { formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2];}
   else if((ros::Time::now()).toSec()>24 && (ros::Time::now()).toSec()<34)
    { formulation_.final_base_.lin.at(towr::kPos) <<formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2]+0.025;}
   else if((ros::Time::now()).toSec()>34 )
    { formulation_.final_base_.lin.at(towr::kPos) << formulation_.initial_base_.lin.at(towr::kPos)[0]+0.04*std::sin(formulation_.initial_base_.ang.at(towr::kPos)[2]), formulation_.initial_base_.lin.at(towr::kPos)[1]-0.04*std::cos(formulation_.initial_base_.ang.at(towr::kPos)[2]), 0.40229;
      formulation_.final_base_.ang.at(towr::kPos) << 0.0, -0.0, formulation_.initial_base_.ang.at(towr::kPos)[2];}


   // Set initial state
   auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();
   formulation_.initial_ee_W_ = nominal_stance_B;  


   Eigen::Vector3d pos_ee;
   for (int ee=0; ee<4; ee++){
     switch(ee){
      case 0: pos_ee=doggo->getBLpos();
      break;
      case 1: pos_ee=doggo->getBRpos();
      break;
      case 2: pos_ee=doggo->getFLpos();
      break;
      case 3: pos_ee=doggo->getFRpos();
      break;
     }

    formulation_.initial_ee_W_.at(ee)[0]=pos_ee[0];
    formulation_.initial_ee_W_.at(ee)[1]=pos_ee[1];
   }

   std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
                   [&](Eigen::Vector3d& p){  p[2]= 0.0; } );
  
  
  // Choose gait
   auto gait_gen_ = towr::GaitGenerator::MakeGaitGenerator(4);
   if (gait_flag==1){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C1);
    gait_gen_->SetCombo(id_gait);
   }
   else if (gait_flag==2){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C5);
    gait_gen_->SetCombo(id_gait);
   }
   else if (gait_flag==3){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C6);
    gait_gen_->SetCombo(id_gait);
   }
   else if (gait_flag==4){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C7);
    gait_gen_->SetCombo(id_gait);
   }
   else if (gait_flag==5){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C8);
    gait_gen_->SetCombo(id_gait);
   }
   else if (gait_flag==6){
    auto id_gait   = static_cast<towr::GaitGenerator::Combos>(towr::GaitGenerator::C9);
    gait_gen_->SetCombo(id_gait);
   }

   formulation_.params_.ee_phase_durations_.clear();
   for (int ee=0; ee<4; ++ee) {
      formulation_.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(0.5, ee));
      formulation_.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
      std::cout<<"ciao"<<std::endl;
    }


  ifopt::Problem nlp;
  
  // Compute solution
  for (auto c : formulation_.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation_.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation_.GetCosts())
    nlp.AddCostSet(c);

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);
   
  return formulation_;

}


// Main 
int main(int argc, char *argv[])
{   
    //Load the urdf and create the quadruped
      std::string modelFile = argv[1];
    
       doggo= new QUADRUPED(modelFile);

    
      if( argc != 2 )
      {
        std::cerr << "KinDynComputationsWithEigen usage: KinDynComputationsWithEigen ./path/to/modelName.urdf" << std::endl;
        return EXIT_FAILURE;
      }

    // Helper class to load the model from an external format
      iDynTree::ModelLoader mdlLoader;

      bool ok = mdlLoader.loadModelFromFile(modelFile);

      if( !ok )
      {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return EXIT_FAILURE;
      }
   
    // Create a KinDynComputations class from the model
     iDynTree::KinDynComputations kinDynComp;
     ok = kinDynComp.loadRobotModel(mdlLoader.model());

     if( !ok )
     {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return EXIT_FAILURE;
     }
    
     const iDynTree::Model & model = kinDynComp.model();

    // Start node
     ros::init(argc, argv, "ros_control_node");
     ros::NodeHandle n;

    // Gazebo node 
     gazebo::transport::NodePtr node(new gazebo::transport::Node());
     node->Init();


    // Rate
      
      gazebo::client::setup(argc,argv);


    // Ros  Subscribers
      ros::Subscriber joint_state_sub = n.subscribe("/dogbot/joint_states", 1, jointStateCallback);
      ros::Subscriber model_state_sub = n.subscribe("/gazebo/model_states", 1, modelStateCallback);

    // Ros publisher
     joint_effort_pub = n.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command", 1);

    // Ros services
     ros::ServiceClient set_model_configuration_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
     ros::ServiceClient set_model_state_srv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
     ros::ServiceClient resetGazeboSimulation = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
     ros::ServiceClient pauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
     ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
     _eebl_sub = n.subscribe("/dogbot/back_left_contactsensor_state",1, eebl_cb);
     _eefl_sub = n.subscribe("/dogbot/front_left_contactsensor_state",1, eefl_cb);
     _eebr_sub = n.subscribe("/dogbot/back_right_contactsensor_state",1, eebr_cb);
     _eefr_sub = n.subscribe("/dogbot/front_right_contactsensor_state",1, eefr_cb);
     _distxbr_sub = n.subscribe("/disturbance_x_br",1, distxbr_cb);
     _distybr_sub = n.subscribe("/disturbance_y_br",1, distybr_cb);
     _distxbl_sub = n.subscribe("/disturbance_x_bl",1, distxbl_cb);
     _distybl_sub = n.subscribe("/disturbance_y_bl",1, distybl_cb);
     _distxfl_sub = n.subscribe("/disturbance_x_fl",1, distxfl_cb);
     _distyfl_sub = n.subscribe("/disturbance_y_fl",1, distyfl_cb);
     _distxfr_sub = n.subscribe("/disturbance_x_fr",1, distxfr_cb);
     _distyfr_sub = n.subscribe("/disturbance_y_fr",1, distyfr_cb);

    // Gazebo publisher in case the qp problem takes too long for the control loop
     pub = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
     pub->WaitForConnection();
    
     // Set multi-step to requested iterations
     stepper.set_step(1);
     ros::Rate loop_rate(1000);
    

    // Start the robot in position (stand up) 
      gazebo_msgs::SetModelConfiguration robot_init_config;
      robot_init_config.request.model_name = "dogbot";
      robot_init_config.request.urdf_param_name = "robot_description";
      robot_init_config.request.joint_names.push_back("back_left_roll_joint");
      robot_init_config.request.joint_names.push_back("back_left_pitch_joint");
      robot_init_config.request.joint_names.push_back("back_left_knee_joint");
      robot_init_config.request.joint_names.push_back("back_right_roll_joint");
      robot_init_config.request.joint_names.push_back("back_right_pitch_joint");
      robot_init_config.request.joint_names.push_back("back_right_knee_joint");
      robot_init_config.request.joint_names.push_back("front_left_roll_joint");
      robot_init_config.request.joint_names.push_back("front_left_pitch_joint");
      robot_init_config.request.joint_names.push_back("front_left_knee_joint");
      robot_init_config.request.joint_names.push_back("front_right_roll_joint");
      robot_init_config.request.joint_names.push_back("front_right_pitch_joint");
      robot_init_config.request.joint_names.push_back("front_right_knee_joint");
      robot_init_config.request.joint_positions.push_back( 0.0004875394147498824);
      robot_init_config.request.joint_positions.push_back( -0.884249947977489);
      robot_init_config.request.joint_positions.push_back(-1.6039026405138666);
      robot_init_config.request.joint_positions.push_back( 0.0006243098169198547);
      robot_init_config.request.joint_positions.push_back(0.8861978063639038);
      robot_init_config.request.joint_positions.push_back(1.6032646991719783);
      robot_init_config.request.joint_positions.push_back(-3.197670677312914e-05);
      robot_init_config.request.joint_positions.push_back(-0.8848124990461947);
      robot_init_config.request.joint_positions.push_back(-1.6039627256817717);
      robot_init_config.request.joint_positions.push_back(-0.0005127385581351618);
      robot_init_config.request.joint_positions.push_back(0.886353788084274);
      robot_init_config.request.joint_positions.push_back( 1.60361055049274);
      if(set_model_configuration_srv.call(robot_init_config))
        ROS_INFO("Robot configuration set.");
      else
        ROS_INFO("Failed to set robot configuration.");
  

      gazebo_msgs::SetModelState robot_init_state;
      robot_init_state.request.model_state.model_name = "dogbot";
      robot_init_state.request.model_state.reference_frame = "world";
      robot_init_state.request.model_state.pose.position.x=-0.00;
      robot_init_state.request.model_state.pose.position.y=-0.034102251365;
      robot_init_state.request.model_state.pose.position.z=0.430159040502;
      robot_init_state.request.model_state.pose.orientation.x=0.0;
      robot_init_state.request.model_state.pose.orientation.y=0.0;
      robot_init_state.request.model_state.pose.orientation.z=0.0;
      robot_init_state.request.model_state.pose.orientation.w=1;
      if(set_model_state_srv.call(robot_init_state))
        ROS_INFO("Robot state set.");
      else
        ROS_INFO("Failed to set robot state.");



    std_srvs::Empty pauseSrv;

     while(!joint_state_available || !base_state_available )
     {
        ROS_INFO_STREAM_ONCE("Robot/object state not available yet.");
        ROS_INFO_STREAM_ONCE("Please start gazebo simulation.");
        ros::spinOnce();
     }
  
    pauseGazebo.call(pauseSrv);
 
    // Update robot
     gravity<<0,0,-9.8;
     
     doggo->update(world_H_base,jnt_pos,jnt_vel,base_vel,gravity);

    // control vector
      tau.resize(12);
      tau= Eigen::VectorXd::Zero(12);

    // Observer vector
      yd_prev.resize(4);  
      yw_prev.resize(4); 
      w_prev.resize(4); 
      ygamma_prev.resize(4);

      yd_prev_sem.resize(4);  
      yw_prev_sem.resize(4); 
      w_prev_sem.resize(4); 
      ygamma_prev_sem.resize(4);

      std::fill(yd_prev.begin(), yd_prev.end(), Eigen::Matrix<double,12,1>::Zero()); 
      std::fill(yw_prev.begin(), yw_prev.end(), Eigen::Matrix<double,12,1>::Zero());
      std::fill(w_prev.begin(), w_prev.end(), Eigen::Matrix<double,12,1>::Zero());
      std::fill(ygamma_prev.begin(), ygamma_prev.end(), Eigen::Matrix<double,12,1>::Zero());

      std::fill(yd_prev_sem.begin(), yd_prev_sem.end(), Eigen::Matrix<double,6,1>::Zero()); 
      std::fill(yw_prev_sem.begin(), yw_prev_sem.end(), Eigen::Matrix<double,6,1>::Zero());
      std::fill(w_prev_sem.begin(), w_prev_sem.end(), Eigen::Matrix<double,6,1>::Zero());
      std::fill(ygamma_prev_sem.begin(), ygamma_prev_sem.end(), Eigen::Matrix<double,6,1>::Zero());


    // Set controller
     controller_= new QUADRUPEDController(*doggo);
     estimator_= new ESTIMATOR(*doggo);

    // Set Gain Matrix
    Kcom=3500*Eigen::MatrixXd::Identity(6,6);
    Dcom=50*Eigen::MatrixXd::Identity(6,6);
   

  // Start locomotion
  while(ros::ok)
 {   

   // Replan trajectory
   formulation=computetrajecotry(1);  
   begin = ros::Time::now();
     ROS_INFO_STREAM_ONCE("Starting control loop ...");

     // Stand phase 
      stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(1)[0]);

    
    ros::Time begin2 = ros::Time::now();
   
    // Swing phase
     flag=0;
     swing_phase( loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);
  
     if (flag==1)
     {swing_phasebr(loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
     else if(flag==2)
     {swing_phasefl(loop_rate, formulation.params_.ee_phase_durations_.at(1)[1]+formulation.params_.ee_phase_durations_.at(1)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
  
 

    // Stand phase
     stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(0)[0]);  
   
     if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
     else
        ROS_INFO("Failed to pause simulation.");  

    // Replan trajectory
     formulation=computetrajecotry(2);

     begin = ros::Time::now();

    // Stand phase
     stand_phase(loop_rate,  formulation.params_.ee_phase_durations_.at(0)[0]);



    // Swing phase
     flag=0;
     swing_phase2( loop_rate, formulation.params_.ee_phase_durations_.at(0)[0]+formulation.params_.ee_phase_durations_.at(0)[1] , formulation.params_.ee_phase_durations_.at(1)[0]);
     if (flag==1)
     {swing_phasefr(loop_rate, formulation.params_.ee_phase_durations_.at(0)[1]+formulation.params_.ee_phase_durations_.at(0)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
     else if(flag==2)
     {swing_phasebl(loop_rate, formulation.params_.ee_phase_durations_.at(0)[1]+formulation.params_.ee_phase_durations_.at(0)[0] , formulation.params_.ee_phase_durations_.at(1)[0]);}
   
   
 

    // Stand phase
     stand_phase(loop_rate,   formulation.params_.ee_phase_durations_.at(1)[0]);

     if(pauseGazebo.call(pauseSrv))
        ROS_INFO("Simulation paused.");
     else
        ROS_INFO("Failed to pause simulation."); 



 }

}