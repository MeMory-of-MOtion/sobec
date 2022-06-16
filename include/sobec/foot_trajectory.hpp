#ifndef MEMMO_FEET_TRAJECTORIES
#define MEMMO_FEET_TRAJECTORIES

#include <ndcurves/bezier_curve.h>
#include <ndcurves/exact_cubic.h>
#include <ndcurves/fwd.h>
#include <ndcurves/piecewise_curve.h>
#include <ndcurves/se3_curve.h>
#include <ndcurves/so3_linear.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <memory>
#include <pinocchio/spatial/se3.hpp>

namespace sobec {

// Eigen typedef
typedef Eigen::Quaternion<double> eQuaternion;
typedef Eigen::Vector2d eVector2;
typedef Eigen::Vector3d eVector3;
typedef Eigen::Matrix<double, 6, 1> eVector6;

typedef Eigen::Matrix2d eMatrixRot2d;
typedef Eigen::Matrix3d eMatrixRot;
typedef Eigen::AngleAxisd eAngleAxis;
}  // namespace sobec

namespace ndcurves {
typedef Eigen::Matrix<double, 1, 1> point1_t;

typedef piecewise_curve<double, double, true, matrix3_t, point3_t, curve_rotation_t> piecewise_SO3_t;
typedef polynomial<double, double, true, point1_t> polynomial1_t;
typedef bezier_curve<double, double, true, point1_t> bezier1_t;
typedef piecewise_curve<double, double, true, point1_t> piecewise1_t;
typedef piecewise_curve<double, double, true, sobec::eVector2> piecewise2_t;
typedef polynomial<double, double, true, sobec::eVector2> polynomial2_t;
typedef constant_curve<double, double, true, sobec::eVector2> constant2_t;
typedef constant_curve<double, double, true, point1_t> constant1_t;

// Template
typedef Eigen::VectorXd pointX_t;
typedef std::vector<pointX_t, Eigen::aligned_allocator<pointX_t> > t_pointX_t;
typedef double time_t;
typedef double num_t;
typedef ndcurves::exact_cubic<double, double, true, pointX_t> exact_cubic_t;
// Example with vector of dimension 3
typedef Eigen::Vector3d point3_t;
// Creation of waypoint container
typedef std::pair<double, pointX_t> Waypoint;
typedef std::vector<Waypoint> T_Waypoint;
}  // namespace ndcurves

namespace sobec {

const double EPSILON_TIME = 1e-9;

// helper geometry struct
struct Spatial {
  static inline double extractYaw(eMatrixRot const& matrix) { return atan2(matrix.data()[1], matrix.data()[0]); }

  static inline double extractYaw(eQuaternion const& q) { return extractYaw(q.toRotationMatrix()); }

  static inline double extractYaw(pinocchio::SE3 const& matrix) { return extractYaw(matrix.rotation()); }

  /// see http://planning.cs.uiuc.edu/node102.html
  static inline eMatrixRot matrixRollPitchYaw(const double& roll, const double& pitch, const double& yaw) {
    double ca(cos(yaw));
    double sa(sin(yaw));

    double cb(cos(pitch));
    double sb(sin(pitch));

    double cc(cos(roll));
    double sc(sin(roll));

    eMatrixRot temp;
    temp << ca * cb, ca * sb * sc - sa * cc, ca * sb * cc + sa * sc, sa * cb, sa * sb * sc + ca * cc,
        sa * sb * cc - ca * sc, -sb, cb * sc, cb * cc;
    return temp;
  }

  static inline eQuaternion quaternionRollPitchYaw(const eVector3& rpy) {
    return eQuaternion(matrixRollPitchYaw(rpy[0], rpy[1], rpy[2]));
  }

  static inline eQuaternion quaternionRollPitchYaw(const double& roll, const double& pitch, const double& yaw) {
    return eQuaternion(matrixRollPitchYaw(roll, pitch, yaw));
  }

  static inline pinocchio::SE3 createMatrix(eQuaternion const& quat, eVector3 const& trans) {
    pinocchio::SE3 temp;
    temp.setIdentity();
    temp.rotation(quat.matrix());
    temp.translation() = trans;
    return temp;
  }
};

class FootTrajectory {
 private:
  typedef ndcurves::Waypoint Waypoint_t;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  /**
   * @brief FootTrajectory
   * @param swing_leg_height desired height at the apex of the trajectory wrt to
   * the average between init and end height
   * @param landing_advance duration before the final time where the feet pose
   * should be reached
   */
  FootTrajectory(const double& swing_leg_height, const double swing_pose_penetration = 0.,
                 const double landing_advance = 0.);
  void generate(const double& t_init, const double& t_end, const pinocchio::SE3& pose_init,
                const pinocchio::SE3& pose_end, const bool& constant = false);
  void update(const double time, const pinocchio::SE3& pose_end);

  double min();
  double max();
  double compute_dt(const double& time);
  pinocchio::SE3 compute(const double& time);
  pinocchio::SE3 start();
  pinocchio::SE3 end();
  eVector3 linear_vel(const double& time);
  eVector3 angular_vel(const double& time);
  eVector3 linear_acc(const double& time);
  eVector3 angular_acc(const double& time);

  const bool is_constant() { return constant_; }

 private:
  pinocchio::SE3 average(const pinocchio::SE3& a, const pinocchio::SE3& b);

  ndcurves::polynomial1_t build_height_predef_trajectory(const double p, const double t_init, const double t_end,
                                                         const double offset, const bool up);

  ndcurves::bezier1_t build_height_predef_lift(const double p, const double t_init, const double t_end);

  ndcurves::bezier1_t build_height_predef_landing(const double p, const double t_init, const double t_end);

  void build_height_trajectory(const double& t_init, const double& t_end, const double& z_init, const double& z_end,
                               const double& v_init = 0., const double& a_init = 0., const double& j_init = 0.);

  void build_height_trajectory_minjerk(const double& t_init, const double& t_end, const double& z_init,
                                       const double& z_end, const double& v_init = 0., const double& a_init = 0.,
                                       const double& j_init = 0.);

  void build_translation_trajectory(const double& t_init, const double& t_end, const eVector2& p_init,
                                    const eVector2& p_end, const eVector2& v_init = eVector2::Zero(),
                                    const eVector2& a_init = eVector2::Zero());

  void build_yaw_trajectory(const double& t_init, const double& t_end, const double& yaw_init, const double& yaw_end,
                            const double& delta_yaw_init = 0., const double& v_init = 0., const double& a_init = 0.);

  ndcurves::bezier1_t build_height_middle_trajectory(const double& t_init, const double& t_end, const double p0,
                                                     const double v0, const double a0, const double j0,
                                                     const double p1, const double v1, const double a1,
                                                     const double j1);

  ndcurves::polynomial2_t build_translation_middle_trajectory(const double& t_init, const double& t_end,
                                                              const eVector2& p_init, const eVector2& p_end,
                                                              const eVector2& v_init, const eVector2& a_init);

  ndcurves::polynomial1_t build_yaw_middle_trajectory(const double& t_init, const double& t_end,
                                                      const double& yaw_init, const double& yaw_end,
                                                      const double& delta_yaw_init, const double& v_init,
                                                      const double& a_init);

 private:
  // Original paramters before end position update.
  pinocchio::SE3 pose_init_, pose_end_;
  double t_init_;
  double t_end_;

  bool constant_;

  pinocchio::SE3 previous_pose_end_;

  // Curves.
  ndcurves::piecewise1_t yaw_traj_;
  ndcurves::piecewise2_t translation_traj_;
  ndcurves::piecewise1_t height_traj_;

  double swing_leg_height_;
  double swing_pose_penetration_;
  double landing_advance_;
  double lift_factor_;
  double land_factor_;
  double lift_offset_;
  double land_offset_;
  double lift_end_time_, land_begin_time_;
  ndcurves::point3_t normal_;
};

typedef std::shared_ptr<FootTrajectory> FootTrajectory_ptr;

}  // namespace sobec

#endif  // MEMMO_FEET_TRAJECTORIES
