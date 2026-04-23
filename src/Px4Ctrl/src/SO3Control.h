#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>
#include <quadrotor_msgs/Px4ctrlDebug.h>

class SO3Control
{
public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d &position);
  void setVelocity(const Eigen::Vector3d &velocity);
  void setAcc(const Eigen::Vector3d &acc);
  void setQuat(const Eigen::Quaterniond &q);

  void calculateControl(const Eigen::Vector3d &des_pos,
                        const Eigen::Vector3d &des_vel,
                        const Eigen::Vector3d &des_acc,
                        const Eigen::Vector3d &des_jer,
                        const Eigen::Vector3d &des_dir,
                        const Eigen::Vector3d &des_dir_dot,
                        const double des_yaw, const double des_yaw_dot,
                        const Eigen::Vector3d &kx,
                        const Eigen::Vector3d &kv,
                        const Eigen::Vector3d &kR,
                        quadrotor_msgs::Px4ctrlDebug &debug_);
  
    // 新增：设置推力上下限（单位同 thr_，通常是 N）
  void setThrustLimits(double thr_min, double thr_max);
  void setOmegaLimits(const Eigen::Vector3d &omega_max);

  bool forward(const Eigen::Vector3d &vel,
               const Eigen::Vector3d &acc,
               const Eigen::Vector3d &jer,
               const Eigen::Vector3d &dir,
               const Eigen::Vector3d &ddir,
               double &psi,
               double &dpsi,
               double &thr_,
               Eigen::Vector4d &quat,
               Eigen::Vector3d &omg);

  const Eigen::Vector3d &getComputedForce(void);
  const Eigen::Quaterniond &getComputedOrientation(void);
  const Eigen::Vector3d &getComputedOmega(void);
  const double &getComputedYaw(void);
  const double &getComputedYawDot(void);
  const double &getThrust(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Inputs for the controller
  double mass_;
  double g_;
  double thr_;

  // 新增
  double thr_min_;   // 控制层允许的最小总推力
  double thr_max_;   // 控制层允许的最大总推力
  Eigen::Vector3d omega_max_;

  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  Eigen::Quaterniond quat_;
  Eigen::Vector3d old_omega_;

  // Outputs of the controller
  double yaw_, yaw_dot_;
  Eigen::Vector3d force_;
  Eigen::Vector3d omega_;
  Eigen::Quaterniond orientation_;
};

#endif
