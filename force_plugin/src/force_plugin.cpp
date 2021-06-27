#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <gazebo/common/Time.hh>
#include <tgmath.h> 
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <Eigen/Core>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {   private: ros::Publisher _distx_pub_br;
      private: ros::Publisher _disty_pub_br;
      private: ros::Publisher _distx_pub_bl;
      private: ros::Publisher _disty_pub_bl;
      private: ros::Publisher _distx_pub_fl;
      private: ros::Publisher _disty_pub_fl;
      private: ros::Publisher _distx_pub_fr;
      private: ros::Publisher _disty_pub_fr;
      private: ros::Publisher _distflag;
      private: int flag;
      private: int flag_sign;
      private: std_msgs::Float64 x_dist;
      private: std_msgs::Float64 y_dist;
      private: ros::NodeHandle* _node_handle;
      private: ignition::transport::Node node_ign;
      private: ignition::msgs::Marker markerMsg;
      private: ignition::msgs::Marker markerMsg2;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {      _node_handle = new ros::NodeHandle();
	
  	_distx_pub_br = _node_handle->advertise<std_msgs::Float64>("/disturbance_x_br", 0);
    _disty_pub_br = _node_handle->advertise<std_msgs::Float64>("/disturbance_y_br", 0);
    _distx_pub_bl = _node_handle->advertise<std_msgs::Float64>("/disturbance_x_bl", 0);
    _disty_pub_bl = _node_handle->advertise<std_msgs::Float64>("/disturbance_y_bl", 0);
    _distx_pub_fl = _node_handle->advertise<std_msgs::Float64>("/disturbance_x_fl", 0);
    _disty_pub_fl = _node_handle->advertise<std_msgs::Float64>("/disturbance_y_fl", 0);
    _distx_pub_fr = _node_handle->advertise<std_msgs::Float64>("/disturbance_x_fr", 0);
    _disty_pub_fr = _node_handle->advertise<std_msgs::Float64>("/disturbance_y_fr", 0);
    
    
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      markerMsg.set_ns("default");
       markerMsg.set_id(0);
      markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
      markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
      ignition::msgs::Set(markerMsg.mutable_scale(),
                    ignition::math::Vector3d(0.065, 0.065, 0.065));
       ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, -1, 0.0));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.0));
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.01));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, -1, 0.01));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.01));

 
   
   
   
      markerMsg2.set_ns("default");
      markerMsg2.set_id(1);
      markerMsg2.set_action(ignition::msgs::Marker::ADD_MODIFY);
      markerMsg2.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
       ignition::msgs::Set(markerMsg2.mutable_scale(),
                    ignition::math::Vector3d(0.16, 0.05, 0.2));
      ignition::msgs::Set(markerMsg2.add_point(),
        ignition::math::Vector3d(0, -0.5, 0.0));
  ignition::msgs::Set(markerMsg2.add_point(),
      ignition::math::Vector3d(1, -0.5, 0.0));
  ignition::msgs::Set(markerMsg2.add_point(),
      ignition::math::Vector3d(0, 0.5, 0.0));
   ignition::msgs::Set(markerMsg2.add_point(),
      ignition::math::Vector3d(1, 0.5, 0.0));

  ignition::msgs::Set(markerMsg2.add_point(),
        ignition::math::Vector3d(0, -0.5, 0.01));
  ignition::msgs::Set(markerMsg2.add_point(),
      ignition::math::Vector3d(1, -0.5, 0.01));
  ignition::msgs::Set(markerMsg2.add_point(),
      ignition::math::Vector3d(0, 0.5, 0.01));
   ignition::msgs::Set(markerMsg2.add_point(),
      ignition::math::Vector3d(1, 0.5, 0.01));

   
 


      ignition::msgs::Material *matMsg = markerMsg.mutable_material();
       ignition::msgs::Material *matMsg2 = markerMsg2.mutable_material();
      matMsg->mutable_script()->set_name("Gazebo/YellowGlow");
      matMsg2->mutable_script()->set_name("Gazebo/YellowGlow");
      
    }





// Called by the world update start event
    public: void OnUpdate()
    {  

    //Case Study 1
      /* common::Time currTime = this->model->GetWorld()->SimTime();
       y_dist.data=0*sin(currTime.Double());
       x_dist.data=20*sin(currTime.Double());
       double z=0;
       _distx_pub_br.publish( x_dist );
       _disty_pub_br.publish( y_dist );
       _back_right_lowerleg = model->GetLink("back_right_lowerleg");
       ignition::math::Vector3d pos=_back_right_lowerleg->WorldPose().Pos();	
       _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d( x_dist.data,0,0),ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853 ,-0.321099633029770));
       double alfa=atan2(y_dist.data, x_dist.data);
       double theta=atan2(z,x_dist.data);
      // if(x_dist.data<0)
       //{ 
       ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.04*cos(3.14+alfa),pos.Y()+0.04*sin(3.14+alfa),pos.Z(),1.57,theta,3.14+alfa));
   
        ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.08*cos(3.14+alfa),pos.Y()+0.08*sin(3.14+alfa),pos.Z(),1.57,theta,3.14+alfa));//}
   
        node_ign.Request("/marker", markerMsg);
        node_ign.Request("/marker", markerMsg2);*/





    // Case Study 3
       /*if (currTime>=2.88 && currTime<=3.1 ) {
        _back_right_lowerleg = model->GetLink("back_left_lowerleg");
        _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(60,0,0),ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853 ,-0.321099633029770));
        x_dist.data=80;	
       _distx_pub_bl.publish( x_dist );
        ignition::math::Vector3d pos=_back_right_lowerleg->WorldPose().Pos();	
        if(x_dist.data<0)
       { 
       ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.06,pos.Y(),pos.Z(),1.57,0.0,0.0));
   
        ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.10,pos.Y(),pos.Z(),1.57,0.0,0.0));}
        else
       { 
       ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()-0.06,pos.Y(),pos.Z(),1.57,3.14,0.0));
   
        ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()-0.10,pos.Y(),pos.Z(),1.57,3.14,0.0));}
        node_ign.Request("/marker", markerMsg);
        node_ign.Request("/marker", markerMsg2);
  
        }
       else    if (currTime<2.88 ) { x_dist.data=0;	
        _distx_pub_bl.publish( x_dist );
       }
       else if (currTime>3.1 ) {
        markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
        markerMsg2.set_action(ignition::msgs::Marker::DELETE_ALL);
        node_ign.Request("/marker", markerMsg);
        node_ign.Request("/marker", markerMsg2);}*/
      

    

     
    
    
  
    
   
    //  Case Studies 4 and 5
       /* if (currTime.Double()==0.001)
         { flag=5;}
          if (fmod(currTime.Double(),2)==0)
         {flag=rand()%4; 
          flag_sign= rand()%4;
         if (flag==0){
         if (flag_sign==0)
         {
          x_dist.data=rand()%20+5;
          y_dist.data=rand()%10+5;}
         else if (flag_sign==1)
        {
         x_dist.data=-(rand()%20+5);
         y_dist.data=rand()%10+5;}
       else if (flag_sign==2)
        {
         x_dist.data=rand()%20+5;
         y_dist.data=-(rand()%10+5);}

        else if (flag_sign==3)
        {
         x_dist.data=-(rand()%20+5);
         y_dist.data=-(rand()%10+5);}

         std_msgs::Float64 x_dist_n;
         std_msgs::Float64 y_dist_n;
         x_dist_n.data=0;
         y_dist_n.data=0;
        _distx_pub_bl.publish( x_dist_n );
        _disty_pub_bl.publish( y_dist_n );

        _distx_pub_fl.publish( x_dist_n );
        _disty_pub_fl.publish( y_dist_n );

        _distx_pub_fr.publish( x_dist_n );
        _disty_pub_fr.publish( y_dist_n );
       }

       else if (flag==1){
         if (flag_sign==0)
       {
        x_dist.data=rand()%10+5;
        y_dist.data=rand()%10+5;}
       else if (flag_sign==1)
      {
       x_dist.data=-(rand()%10+5);
       y_dist.data=rand()%10+5;}
      else if (flag_sign==2)
      {
       x_dist.data=rand()%10+5;
       y_dist.data=-(rand()%10+5);}
      else if (flag_sign==3)
      {
       x_dist.data=-(rand()%10+5);
       y_dist.data=-(rand()%10+5);}
       std_msgs::Float64 x_dist_n;
       std_msgs::Float64 y_dist_n;
       x_dist_n.data=0;
       y_dist_n.data=0;
        _distx_pub_br.publish( x_dist_n );
      _disty_pub_br.publish( y_dist_n );
      _distx_pub_fl.publish( x_dist_n );
     _disty_pub_fl.publish( y_dist_n );
     _distx_pub_fr.publish( x_dist_n );
     _disty_pub_fr.publish( y_dist_n );
     }
     
     else if (flag==2){
       if (flag_sign==0)
      {
       x_dist.data=rand()%20+5;
       y_dist.data=rand()%5;}
     else if (flag_sign==1)
     {
      x_dist.data=-(rand()%20+5);
      y_dist.data=rand()%5;}
     else if (flag_sign==2)
    {
     x_dist.data=rand()%20+5;
     y_dist.data=-(rand()%5);}
     else if (flag_sign==3)
    {
     x_dist.data=-(rand()%20+5);
     y_dist.data=-(rand()%5);}
     std_msgs::Float64 x_dist_n;
     std_msgs::Float64 y_dist_n;
     x_dist_n.data=0;
     y_dist_n.data=0;
     _distx_pub_bl.publish( x_dist_n );
     _disty_pub_bl.publish( y_dist_n );
     _distx_pub_fl.publish( x_dist_n );
     _disty_pub_fl.publish( y_dist_n );
     _distx_pub_br.publish( x_dist_n );
     _disty_pub_br.publish( y_dist_n );
     }

     else if (flag==3){
      if (flag_sign==0)
      {
       x_dist.data=rand()%10+5;
       y_dist.data=rand()%10+5;}
     else if (flag_sign==1)
      {
       x_dist.data=-(rand()%10+5);
       y_dist.data=rand()%10+5;}
     else if (flag_sign==2)
      {
       x_dist.data=rand()%10+5;
       y_dist.data=-(rand()%10+5);}
     else if (flag_sign==3)
      {
       x_dist.data=-(rand()%10+5);
       y_dist.data=-(rand()%10+5);}
      std_msgs::Float64 x_dist_n;
      std_msgs::Float64 y_dist_n;
      x_dist_n.data=0;
      y_dist_n.data=0;
      _distx_pub_bl.publish( x_dist_n );
     _disty_pub_bl.publish( y_dist_n );
     _distx_pub_br.publish( x_dist_n );
     _disty_pub_br.publish( y_dist_n );
     _distx_pub_fr.publish( x_dist_n );
     _disty_pub_fr.publish( y_dist_n );

     }
    }
    

     if (flag==0)
     { //_back_right_lowerleg = model->GetLink("back_right_lowerleg");
     _back_right_lowerleg = model->GetLink("back_right_lowerleg");
     double t = (ros::Time::now()).toSec();
    
    // _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853, -0.17));
     _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853 ,-0.321099633029770));
    
     ignition::math::Vector3d pos=_back_right_lowerleg->WorldPose().Pos()+_back_right_lowerleg->WorldPose().Rot().RotateVector(ignition::math::Vector3d(2.059009593607686e-05, -0.016585135985853, -0.321099633029770));
     _distx_pub_br.publish( x_dist );
     _disty_pub_br.publish( y_dist );
 
     std_msgs::Float64 x_dist_n;
    std_msgs::Float64 y_dist_n;
     x_dist_n.data=0;
    
     y_dist_n.data=0;
        _distx_pub_fr.publish( x_dist_n );
     _disty_pub_fr.publish( y_dist_n );

   _distx_pub_bl.publish( x_dist_n );
     _disty_pub_bl.publish( y_dist_n );

   _distx_pub_fl.publish( x_dist_n );
     _disty_pub_fl.publish( y_dist_n );
       double alfa=atan2(y_dist.data, x_dist.data);
    // if(x_dist.data<0)
      //{ 
     ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.04*cos(3.14+alfa),pos.Y()+0.04*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));
   
      ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.08*cos(3.14+alfa),pos.Y()+0.08*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));//}
   
      node_ign.Request("/marker", markerMsg);
      node_ign.Request("/marker", markerMsg2);
    }

     else if (flag==1)
    { //_back_right_lowerleg = model->GetLink("back_left_upperleg");
    _back_right_lowerleg = model->GetLink("back_left_upperleg");
    double t = (ros::Time::now()).toSec();
   
    //_back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(0,0,0));
    _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(0,0,-0.17));
    
	
     _distx_pub_bl.publish( x_dist );
     _disty_pub_bl.publish( y_dist );
      ignition::math::Vector3d pos=_back_right_lowerleg->WorldPose().Pos()+_back_right_lowerleg->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0,0,-0.17));

      std_msgs::Float64 x_dist_n;
    std_msgs::Float64 y_dist_n;
     x_dist_n.data=0;
    
     y_dist_n.data=0;
        _distx_pub_br.publish( x_dist_n );
     _disty_pub_br.publish( y_dist_n );

   _distx_pub_fr.publish( x_dist_n );
     _disty_pub_fr.publish( y_dist_n );

   _distx_pub_fl.publish( x_dist_n );
     _disty_pub_fl.publish( y_dist_n );

       double alfa=atan2(y_dist.data, x_dist.data);
    // if(x_dist.data<0)
      //{ 
     ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.04*cos(3.14+alfa),pos.Y()+0.04*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));
   
      ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.08*cos(3.14+alfa),pos.Y()+0.08*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));//}
   
      node_ign.Request("/marker", markerMsg);
      node_ign.Request("/marker", markerMsg2);

    }

     else if (flag==2)
    { _back_right_lowerleg = model->GetLink("front_right_lowerleg");
   //_back_right_lowerleg = model->GetLink("back_left_upperleg");
    double t = (ros::Time::now()).toSec();
  
    //x_dist.data=20*sin(currTime.Double());
     
   //    _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(0,0,-0.17));
    _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(0,0,0));
     _distx_pub_fr.publish( x_dist );
     _disty_pub_fr.publish( y_dist );
     ignition::math::Vector3d pos=_back_right_lowerleg->WorldPose().Pos()+_back_right_lowerleg->WorldPose().Rot().RotateVector(ignition::math::Vector3d(0,0,0));

      std_msgs::Float64 x_dist_n;
    std_msgs::Float64 y_dist_n;
     x_dist_n.data=0;
    
     y_dist_n.data=0;
        _distx_pub_br.publish( x_dist_n );
     _disty_pub_br.publish( y_dist_n );

   _distx_pub_bl.publish( x_dist_n );
     _disty_pub_bl.publish( y_dist_n );

   _distx_pub_fl.publish( x_dist_n );
     _disty_pub_fl.publish( y_dist_n );

       double alfa=atan2(y_dist.data, x_dist.data);
    // if(x_dist.data<0)
      //{ 
     ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.04*cos(3.14+alfa),pos.Y()+0.04*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));
   
      ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.08*cos(3.14+alfa),pos.Y()+0.08*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));//}
   
      node_ign.Request("/marker", markerMsg);
      node_ign.Request("/marker", markerMsg2);
     }
   

    else  if (flag==3)
    { _back_right_lowerleg = model->GetLink("front_left_lowerleg");

    double t = (ros::Time::now()).toSec();
 
     //_back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853, -0.17));
   
       _back_right_lowerleg->AddForceAtRelativePosition(ignition::math::Vector3d(x_dist.data,y_dist.data,0),ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853, -0.17));
      
 
       ignition::math::Vector3d pos=_back_right_lowerleg->WorldPose().Pos()+_back_right_lowerleg->WorldPose().Rot().RotateVector(ignition::math::Vector3d(-2.059009593607686e-05, -0.016585135985853,-0.17));	
	
     _distx_pub_fl.publish( x_dist );
     _disty_pub_fl.publish( y_dist );

       std_msgs::Float64 x_dist_n;
    std_msgs::Float64 y_dist_n;
     x_dist_n.data=0;
    
     y_dist_n.data=0;
        _distx_pub_br.publish( x_dist_n );
     _disty_pub_br.publish( y_dist_n );

   _distx_pub_bl.publish( x_dist_n );
     _disty_pub_bl.publish( y_dist_n );

   _distx_pub_fr.publish( x_dist_n );
     _disty_pub_fr.publish( y_dist_n );
      
       double alfa=atan2(y_dist.data, x_dist.data);
    // if(x_dist.data<0)
      //{ 
     ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.04*cos(3.14+alfa),pos.Y()+0.04*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));
   J=
      ignition::msgs::Set(markerMsg2.mutable_pose(),
                      ignition::math::Pose3d(pos.X()+0.08*cos(3.14+alfa),pos.Y()+0.08*sin(3.14+alfa),pos.Z(),1.57,0.0,3.14+alfa));//}
   
      node_ign.Request("/marker", markerMsg);
      node_ign.Request("/marker", markerMsg2);
    }*/

   


    }


    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    private: physics::LinkPtr  _back_right_lowerleg;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
