#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_DOGBOT_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_DOGBOT_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot ANYmal.
 */
class DogbotKinematicModel : public KinematicModel {
public:
  DogbotKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b_lf = 0.186571;
    const double y_nominal_b_lf = 0.289186;
    const double z_nominal_b_lf = -0.40229;

    const double x_nominal_b_rf = 0.186988;
    const double y_nominal_b_rf = 0.2882;
    const double z_nominal_b_rf = -0.40229;

    const double x_nominal_b_lr = 0.186767;
    const double y_nominal_b_lr = 0.286043;
    const double z_nominal_b_lr = -0.40229;

    const double x_nominal_b_rr = 0.18694;
    const double y_nominal_b_rr = 0.286787;
    const double z_nominal_b_rr = -0.40229;

   /* const double base_delta_x = 0.0;
    const double base_delta_y = 0.0212;
    const double base_delta_z = 0.03;*/

   /* const double x_nominal_b_lf = 0.18656;
    const double y_nominal_b_lf = 0.302086;
    const double z_nominal_b_lf = -0.402152;

    const double x_nominal_b_rf = 0.186998;
    const double y_nominal_b_rf = 0.3011;
    const double z_nominal_b_rf = -0.402152;

    const double x_nominal_b_lr = 0.186756;
    const double y_nominal_b_lr = 0.273143;
    const double z_nominal_b_lr = -0.402152;

    const double x_nominal_b_rr = 0.18695;
    const double y_nominal_b_rr = 0.273887;
    const double z_nominal_b_rr = -0.402152;*/

    const double base_delta_x = 0.0;
    const double base_delta_y = 0.0;
    const double base_delta_z = 0.0;


    nominal_stance_.at(LF) <<  -x_nominal_b_lf,   -y_nominal_b_lf-base_delta_y, z_nominal_b_lf;//+base_delta_z;
    nominal_stance_.at(RF) <<  x_nominal_b_rf,   -y_nominal_b_rf-base_delta_y, z_nominal_b_rf;//+base_delta_z;
    nominal_stance_.at(LH) << -x_nominal_b_lr,   y_nominal_b_lr-base_delta_y, z_nominal_b_lr;//+base_delta_z;
    nominal_stance_.at(RH) << x_nominal_b_rr,  y_nominal_b_rr-base_delta_y, z_nominal_b_rr;//+base_delta_z;

    max_dev_from_nominal_ << 0.1, 0.15, 0.05;
  }
};

/**
 * @brief The Dynamics of the quadruped robot ANYmal.
 */
class DogbotDynamicModel : public SingleRigidBodyDynamics {
public:
  DogbotDynamicModel()
  : SingleRigidBodyDynamics(21.261,
                    1.6375, 0.7098, 2.0399, -0.000291252, -0.000179158, 0.0737803,
                    4) {}
};

} // namespace towr

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_DOGBOT_MODEL_H_ */
