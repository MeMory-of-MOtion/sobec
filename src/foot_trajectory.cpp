//#include <ros/ros.h>
#include "sobec/foot_trajectory.hpp"
namespace sobec
{
  FootTrajectory::FootTrajectory(const double& swing_leg_height,
                                const double swing_pose_penetration, const double landing_advance)
    : swing_leg_height_(swing_leg_height)
    , swing_pose_penetration_(swing_pose_penetration)
    , landing_advance_(landing_advance)
  {
    ;
    lift_factor_ = 1.0 / 8.;
    land_factor_ = 1.0 / 8.;
    lift_offset_ = 0.05 * swing_leg_height_;
    land_offset_ = 0.05 * swing_leg_height_;
    normal_ = Eigen::Vector3d::UnitZ();
  }

  double compute_required_jerk(const double offset, const double time)
  {
    return offset / (time * time * time);
  }

  ndcurves::polynomial1_t FootTrajectory::build_height_predef_trajectory(
      const double p, const double t_init, const double t_end, const double offset, const bool up)
  {
    // build a constant jerk trajectory.
    // Either starting from pose height with 0 velocity and acceleration if up == true
    // Or ending here if up == false
    // the jerk is computed such that the curve move of 'offset' for the desired duration
    const double duration = t_end - t_init;
    const double jerk_coef = compute_required_jerk(offset, duration);  // it actually jerk/6

    Eigen::MatrixXd coeffs(1, 4);
    if (up)
    {
      coeffs(0, 0) = p;
      coeffs(0, 1) = 0.;  // initial vel;
      coeffs(0, 2) = 0.;  // initial acc
      coeffs(0, 3) = jerk_coef;
    }
    else
    {
      coeffs(0, 0) = p + jerk_coef * duration * duration * duration;
      coeffs(0, 1) = -3 * jerk_coef * duration * duration;
      coeffs(0, 2) = 3 * jerk_coef * duration;
      coeffs(0, 3) = -jerk_coef;
    }
    return ndcurves::polynomial1_t(coeffs, t_init, t_end);
  }


  double compute_predef_height(const double duration_predef, const double t_middle, const double p_max)
  {
    double p = p_max / (1. + 4. * t_middle / duration_predef +
                        6. * t_middle * t_middle / (duration_predef * duration_predef) -
                        (t_middle * t_middle * t_middle) /
                            (duration_predef * duration_predef * duration_predef));
    if (p < 0.)
    {
      p = std::abs(p);  // FIXME : why/when does it happen ? eg. with t_total = 3.4 and t_predef = 0.2
    }

    return p;
  }
  ndcurves::bezier1_t FootTrajectory::build_height_predef_lift(const double p_init,
                                                              const double t_init,
                                                              const double t_end)
  {
    /*
    * Build a bezier of degree 4 starting at (p, 0, 0, 0) and going to p + offset with
    * unconstrained derivatives control points are computed as follow:
    *
    * wps[:, 0] = (c0)
    * wps[:, 1] = ((dc0 * T / n) + c0)
    * wps[:, 2] = ((n * n * c0 - n * c0 + 2. * n * dc0 * T - 2. * dc0 * T + ddc0 * T * T) /
    * (n * (n - 1.))) wps[:, 3] = ((n * n * c0 - n * c0 + 3. * n * dc0 * T - 3. * dc0 * T
    * + 3. * ddc0 * T * T) / (n * (n - 1.))) wps[:, 4] = (c1)
    */
    const double T(t_end - t_init);
    const double p_end = p_init + compute_predef_height(T, t_end_ - t_init_, lift_offset_);
    const int n(4);  // degree of this curve
    ndcurves::bezier1_t::t_point_t coeffs(n + 1);
    coeffs[0] = ndcurves::point1_t(p_init);
    coeffs[1] = ndcurves::point1_t(p_init);
    coeffs[2] = ndcurves::point1_t(p_init);
    coeffs[3] = ndcurves::point1_t(p_init);
    coeffs[4] = ndcurves::point1_t(p_end);

    return ndcurves::bezier1_t(coeffs.begin(), coeffs.end(), t_init, t_end);
  }

  ndcurves::bezier1_t FootTrajectory::build_height_predef_landing(const double p_end,
                                                                  const double t_init,
                                                                  const double t_end)
  {
    /*
    * Build a bezier of degree 4 starting at (p - offset) with unconstrained derivatives
    * and going to (p, 0, 0, 0) control points are computed as follow:
    *
    * wps[:, 0] = (c0)
    * wps[:, 1] = ((n * n * c1 - n * c1 - 3 * n * dc1 * T + 3 * dc1 * T + 3 * ddc1 * T * T)
    * / (n * (n - 1))) wps[:, 2] = ((n * n * c1 - n * c1 - 2 * n * dc1 * T + 2 * dc1 * T +
    * ddc1 * T * T) / (n * (n - 1))) wps[:, 3] = ((-dc1 * T / n) + c1) wps[:, 4] = (c1)
    */

    const double T(t_end - t_init);
    const double p_init = p_end + compute_predef_height(T, t_end_ - t_init_, land_offset_);
    const int n(4);  // degree of this curve
    ndcurves::bezier1_t::t_point_t coeffs(n + 1);
    coeffs[0] = ndcurves::point1_t(p_init);
    coeffs[1] = ndcurves::point1_t(p_end);
    coeffs[2] = ndcurves::point1_t(p_end);
    coeffs[3] = ndcurves::point1_t(p_end);
    coeffs[4] = ndcurves::point1_t(p_end);

    return ndcurves::bezier1_t(coeffs.begin(), coeffs.end(), t_init, t_end);
  }

  ndcurves::bezier1_t FootTrajectory::build_height_middle_trajectory(
      const double& t_init, const double& t_end, const double p0, const double v0, const double a0,
      const double j0, const double p1, const double v1, const double a1, const double j1)
  {
    // build a 1D bezier curve of degree 7 connecting exactly the positions, velocity, acceleration and jerk given
    const double T = t_end - t_init;
    const int n = 7;
    ndcurves::bezier1_t::t_point_t coeffs(n + 1);

    coeffs[0] = ndcurves::point1_t(p0);
    coeffs[1] = ndcurves::point1_t((v0 * T / n) + p0);
    coeffs[2] = ndcurves::point1_t(
        (n * n * p0 - n * p0 + 2. * n * v0 * T - 2. * v0 * T + a0 * T * T) / (n * (n - 1.)));
    coeffs[3] = ndcurves::point1_t((n * n * p0 - n * p0 + 3. * n * v0 * T - 3. * v0 * T +
                                    3. * a0 * T * T + j0 * T * T * T / (n - 2)) /
                                  (n * (n - 1.)));
    coeffs[4] = ndcurves::point1_t((n * n * p1 - n * p1 - 3 * n * v1 * T + 3 * v1 * T +
                                    3 * a1 * T * T - j1 * T * T * T / (n - 2)) /
                                  (n * (n - 1)));
    coeffs[5] = ndcurves::point1_t(
        (n * n * p1 - n * p1 - 2 * n * v1 * T + 2 * v1 * T + a1 * T * T) / (n * (n - 1)));
    coeffs[6] = ndcurves::point1_t((-v1 * T / n) + p1);
    coeffs[7] = ndcurves::point1_t(p1);

    return ndcurves::bezier1_t(coeffs.begin(), coeffs.end(), t_init, t_end);
  }

  void FootTrajectory::build_height_trajectory_minjerk(const double& t_init,
                                                      const double& t_end, const double& z_init,
                                                      const double& z_end, const double& v_init,
                                                      const double& a_init, const double& j_init)
  {
    height_traj_ = ndcurves::piecewise1_t();
    const double mid_time = (t_init_ + t_end_) / 2.; // time at the apex for the initial trajectory (before any call to update)
    const double z_apex = z_init + swing_leg_height_;
    if(v_init == 0. && a_init == 0. && j_init == 0.){
      if(t_init < mid_time - 0.1) // - 0.1 to avoid too short time for reaching 0 vel and acc. @TODO take that as param
      {
        height_traj_.add_curve(ndcurves::polynomial1_t::MinimumJerk(ndcurves::point1_t(z_init), ndcurves::point1_t(z_apex), t_init, mid_time));
        height_traj_.add_curve(ndcurves::polynomial1_t::MinimumJerk(ndcurves::point1_t(z_apex), ndcurves::point1_t(z_end), mid_time, t_end));
      }
      else{
        height_traj_.add_curve(ndcurves::polynomial1_t::MinimumJerk(ndcurves::point1_t(z_init), ndcurves::point1_t(z_end), t_init, t_end));
      }
    }
    else
    {
      //initial velocity, acceleration and jerk provided. Cannot use minjerk formulation:
      if(t_init < mid_time - 0.1) // - 0.1 to avoid too short time for reaching 0 vel and acc. @TODO take that as param
      {
        height_traj_.add_curve(build_height_middle_trajectory(t_init, mid_time, z_init, v_init, a_init, j_init, z_apex, 0., 0., 0.));
        height_traj_.add_curve(ndcurves::polynomial1_t::MinimumJerk(ndcurves::point1_t(z_apex), ndcurves::point1_t(z_end), mid_time, t_end));
      }
      else{
        height_traj_.add_curve(build_height_middle_trajectory(t_init, t_end, z_init, v_init, a_init, j_init, z_end, 0., 0., 0.));
      }

    }
  }

  void FootTrajectory::build_height_trajectory(const double& t_init, const double& t_end,
                                              const double& z_init, const double& z_end,
                                              const double& v_init, const double& a_init,
                                              const double& j_init)
  {
    height_traj_ = ndcurves::piecewise1_t();

    // build lift and land trajectories :
    // ndcurves::bezier1_t lift_traj, land_traj;
    ndcurves::polynomial1_t lift_traj, land_traj;

    if (t_init < lift_end_time_)
    {
      // lift_traj = build_height_predef_lift(pose_init_.translation().z(), t_init_,
      // lift_end_time_); // always start from initial state, even after update
      lift_traj = build_height_predef_trajectory(pose_init_.translation().z(), t_init_,
                                                lift_end_time_, lift_offset_, true);
      // add trajectories to the final curve :
      height_traj_.add_curve(lift_traj);
    }

    // land_traj = build_height_predef_landing(z_end, land_begin_time_, t_end - landing_advance_);
    land_traj = build_height_predef_trajectory(z_end, land_begin_time_,
                                              t_end - landing_advance_, land_offset_, false);



    // get initial and final position, velocity and acceleration for the middle trajectory :
    double p0, v0, a0, j0;
    double t_begin_middle;
    if (t_init < lift_end_time_)
    {
      p0 = lift_traj(lift_traj.max())[0];
      v0 = lift_traj.derivate(lift_traj.max(), 1)[0];
      a0 = lift_traj.derivate(lift_traj.max(), 2)[0];
      j0 = lift_traj.derivate(lift_traj.max(), 3)[0];
      t_begin_middle = lift_end_time_;
    }
    else
    {
      p0 = z_init;
      v0 = v_init;
      a0 = a_init;
      j0 = j_init;
      t_begin_middle = t_init;
    }
    const double p1 = land_traj(land_traj.min())[0];
    const double v1 = land_traj.derivate(land_traj.min(), 1)[0];
    const double a1 = land_traj.derivate(land_traj.min(), 2)[0];
    const double j1 = land_traj.derivate(land_traj.min(), 3)[0];

    height_traj_.add_curve(build_height_middle_trajectory(t_begin_middle, land_begin_time_,
                                                          p0, v0, a0, j0, p1, v1, a1, j1));
    height_traj_.add_curve(land_traj);
    // add final constant :
    if (height_traj_.max() < t_end)
    {
      const ndcurves::point1_t height_land(land_traj(land_traj.max()));
      height_traj_.add_curve(
          ndcurves::polynomial1_t(height_land, height_land, height_traj_.max(), t_end));
    }
/*
    ROS_DEBUG_STREAM("Height traj, p0 = " << z_init << " p1 = " << z_end);
    ROS_DEBUG_STREAM("init lift point : p="
                    << lift_traj(lift_traj.min())
                    << " v = " << lift_traj.derivate(lift_traj.min(), 1)
                    << " a = " << lift_traj.derivate(lift_traj.min(), 2)
                    << " j = " << lift_traj.derivate(lift_traj.min(), 3));
    ROS_DEBUG_STREAM("end lift point : p=" << lift_traj(lift_traj.max())
                                          << " v = " << lift_traj.derivate(lift_traj.max(), 1)
                                          << " a = " << lift_traj.derivate(lift_traj.max(), 2)
                                          << " j = " << lift_traj.derivate(lift_traj.max(), 3));
    ROS_DEBUG_STREAM("init land point : p="
                    << land_traj(land_traj.min())
                    << " v = " << land_traj.derivate(land_traj.min(), 1)
                    << " a = " << land_traj.derivate(land_traj.min(), 2)
                    << " j = " << land_traj.derivate(land_traj.min(), 3));
    ROS_DEBUG_STREAM("end land point : p=" << land_traj(land_traj.max())
                                          << " v = " << land_traj.derivate(land_traj.max(), 1)
                                          << " a = " << land_traj.derivate(land_traj.max(), 2)
                                          << " j = " << land_traj.derivate(land_traj.max(), 3));*/
  }

  void FootTrajectory::build_translation_trajectory(const double& t_init, const double& t_end,
                                                    const eVector2& p_init, const eVector2& p_end,
                                                    const eVector2& v_init, const eVector2& a_init)
  {
    double t_begin_middle;
    if (t_init < lift_end_time_)
      t_begin_middle = lift_end_time_;
    else
      t_begin_middle = t_init;

    ndcurves::polynomial2_t translation_traj(build_translation_middle_trajectory(
        t_begin_middle, land_begin_time_, p_init, p_end, v_init, a_init));

    // add constant part at the beginning and end if required :
    translation_traj_ = ndcurves::piecewise2_t();
    if (translation_traj.min() > t_init)
    {
      translation_traj_.add_curve(ndcurves::constant2_t(p_init, t_init, translation_traj.min()));
    }
    translation_traj_.add_curve(translation_traj);

    if (translation_traj.max() < t_end)
    {
      translation_traj_.add_curve(ndcurves::constant2_t(p_end, translation_traj.max(), t_end));
    }
  }
  void FootTrajectory::build_yaw_trajectory(const double& t_init, const double& t_end,
                                            const double& yaw_init, const double& yaw_end,
                                            const double& delta_yaw_init,
                                            const double& v_init, const double& a_init)
  {
    double t_begin_middle;
    if (t_init < lift_end_time_)
      t_begin_middle = lift_end_time_;
    else
      t_begin_middle = t_init;
    ndcurves::polynomial1_t yaw_traj(build_yaw_middle_trajectory(
        t_begin_middle, land_begin_time_, yaw_init, yaw_end, delta_yaw_init, v_init, a_init));

    // add constant part at the beginning and end if required :
    yaw_traj_ = ndcurves::piecewise1_t();

    if (yaw_traj.min() > t_init)
    {
      const ndcurves::point1_t zero_yaw(0.);
      yaw_traj_.add_curve(ndcurves::constant1_t(zero_yaw, t_init, yaw_traj.min()));
    }

    yaw_traj_.add_curve(yaw_traj);

    if (yaw_traj.max() < t_end)
    {
      const ndcurves::point1_t end_yaw(yaw_traj(yaw_traj.max()));
      yaw_traj_.add_curve(ndcurves::constant1_t(end_yaw, yaw_traj.max(), t_end));
    }
  }

  ndcurves::polynomial2_t FootTrajectory::build_translation_middle_trajectory(
      const double& t_init, const double& t_end, const eVector2& p_init,
      const eVector2& p_end, const eVector2& v_init, const eVector2& a_init)
  {
    if (v_init.isZero() && a_init.isZero())
      return ndcurves::polynomial2_t::MinimumJerk(p_init, p_end, t_init, t_end);
    else
    {
      return ndcurves::polynomial2_t(p_init, v_init, a_init, p_end, eVector2::Zero(),
                                    eVector2::Zero(), t_init, t_end);
    }
  }

  ndcurves::polynomial1_t FootTrajectory::build_yaw_middle_trajectory(
      const double& t_init, const double& t_end, const double& yaw_init, const double& yaw_end,
      const double& delta_yaw_init, const double& v_init, const double& a_init)
  {
    // This is relative yaw from the initial pose
    double delta_yaw_end;

    // compute final delta yaw :
    // deal with change of sign/cadrant here
    if (yaw_end - yaw_init < -M_PI)
    {
      delta_yaw_end = (yaw_end + 2. * M_PI) - yaw_init;
    }
    else if (yaw_end - yaw_init > M_PI)
    {
      delta_yaw_end = (yaw_end) - (yaw_init + 2. * M_PI);
    }
    else
    {
      delta_yaw_end = yaw_end - yaw_init;
    }

    if (v_init == 0. && a_init == 0.)
      return ndcurves::polynomial1_t::MinimumJerk(
          ndcurves::point1_t(delta_yaw_init), ndcurves::point1_t(delta_yaw_end), t_init, t_end);
    else
      return ndcurves::polynomial1_t(
          ndcurves::point1_t(delta_yaw_init), ndcurves::point1_t(v_init),
          ndcurves::point1_t(a_init), ndcurves::point1_t(delta_yaw_end),
          ndcurves::point1_t::Zero(), ndcurves::point1_t::Zero(), t_init, t_end);
  }



  void FootTrajectory::generate(const double& t_init, const double& t_end,
                                const pinocchio::SE3 &pose_init, const pinocchio::SE3 &pose_end,
                                const bool& constant)
  {
    assert(t_end > t_init && "FootTrajectory::generateFeetTraj(): t_end > t_init");
    t_init_ = t_init;
    t_end_ = t_end;
    pose_init_ = pose_init;
    pose_end_ = pose_end;
    const double duration = t_end - t_init;
    constant_ = constant;
    // Translation
    if (!constant)
    {
      lift_end_time_ = t_init_ + duration * lift_factor_;
      land_begin_time_ = t_end_ - landing_advance_ - duration * land_factor_;
      double z_end(pose_end.translation().z() - swing_pose_penetration_);
      build_height_trajectory_minjerk(t_init, t_end, pose_init.translation().z(), z_end);
      const eVector2 xy_init(pose_init.translation().x(), pose_init.translation().y()),
          xy_end(pose_end.translation().x(), pose_end.translation().y());
      build_translation_trajectory(t_init, t_end, xy_init, xy_end);
      build_yaw_trajectory(t_init, t_end, Spatial::extractYaw(pose_init),
                          Spatial::extractYaw(pose_end));
    }

/*
    if (constant)
    {
      ROS_DEBUG_STREAM("Generate CONSTANT feet trajectory for t = ["
                      << t_init << " ; " << t_end << "] between \n "
                      << pose_init.translation().transpose() << " and "
                      << pose_end.translation().transpose());
    }
    else
    {
      ROS_DEBUG_STREAM("Generate CONTINUOUS feet trajectory for t = ["
                      << t_init << " ; " << t_end << "] between \n "
                      << pose_init.translation().transpose() << " and "
                      << pose_end.translation().transpose());
      ROS_DEBUG_STREAM("Height trajectory defined for t = [" << height_traj_.min() << " ; "
                                                            << height_traj_.max() << "]");
      ROS_DEBUG_STREAM("XY     trajectory defined for t = ["
                      << translation_traj_.min() << " ; " << translation_traj_.max() << "]");
      ROS_DEBUG_STREAM("Yaw    trajectory defined for t = [" << yaw_traj_.min() << " ; "
                                                            << yaw_traj_.max() << "]");
    }*/
  }

  void FootTrajectory::update(const double time, const pinocchio::SE3 &new_pose_end)
  {
    if (constant_)
    {
      throw std::runtime_error("FootTrajectory::update : Cannot update a constant trajectory.");
    }
    if (time < min() || time > max())
    {
      throw std::runtime_error("FootTrajectory::update : new initial time is out of range");
    }
    // get previous goal and check if it's far from the new one :
    previous_pose_end_ = end();
    if (abs(new_pose_end.translation().x() - previous_pose_end_.translation().x()) >= 0.01 ||
        abs(new_pose_end.translation().y() - previous_pose_end_.translation().y()) >= 0.01)
    {/*
      ROS_WARN_STREAM("!!! Update feet trajectory at time "
                      << time << " from " << previous_pose_end_.translation().transpose()
                      << " to " << new_pose_end.translation().transpose());*/
      if (time >= land_begin_time_)
      {/*
        ROS_ERROR("Calling update footTrajectory after the predefined landing trajectory time. Ignoring the change of target");
        // Should we throw an error here ??*/
        return;
      }
      pose_end_ = new_pose_end;

      // Get the state at t :
      const double init_yaw(yaw_traj_(time)[0]);
      const double init_yaw_d(yaw_traj_.derivate(time, 1)[0]);
      const double init_yaw_dd(yaw_traj_.derivate(time, 2)[0]);
      const double init_height(height_traj_(time)[0]);
      const double init_height_d(height_traj_.derivate(time, 1)[0]);
      const double init_height_dd(height_traj_.derivate(time, 2)[0]);
      const double init_height_ddd(height_traj_.derivate(time, 2)[0]);
      const eVector2 init_xy(translation_traj_(time));
      const eVector2 init_xy_d(translation_traj_.derivate(time, 1));
      const eVector2 init_xy_dd(translation_traj_.derivate(time, 2));

      // create the next trajectories as empty piecewise (do not replace the menbers yet):
      build_yaw_trajectory(time, t_end_, Spatial::extractYaw(pose_init_),
                          Spatial::extractYaw(pose_end_), init_yaw, init_yaw_d, init_yaw_dd);
      build_translation_trajectory(time, t_end_, init_xy, pose_end_.translation().head<2>(),
                                  init_xy_d, init_xy_dd);

      build_height_trajectory_minjerk(time, t_end_, init_height, pose_end_.translation().z(),
                              init_height_d, init_height_dd, init_height_ddd);
    }
  }

  double FootTrajectory::min()
  {
    if (constant_)
      return t_init_;
    else
      return translation_traj_.min();
  }

  double FootTrajectory::max()
  {
    if (constant_)
      return t_end_;
    else
      return translation_traj_.max();
  }

  double FootTrajectory::compute_dt(const double& time)
  {
    double dt = 0.0;
    if (time < translation_traj_.min())
    {
      dt = EPSILON_TIME;
    }
    if (time > translation_traj_.max())
    {
      dt = -EPSILON_TIME;
    }
    return dt;
  }

  pinocchio::SE3 FootTrajectory::compute(const double& time)
  {
    if (constant_)
    {
      return pose_init_;
    }
    pinocchio::SE3 ret(pinocchio::SE3::Identity());
    const double dt = compute_dt(time);
    // compute orientation from delta yaw :
    const double yaw(yaw_traj_(time + dt)[0]);
    ret.rotation(pose_init_.rotation() * Spatial::matrixRollPitchYaw(0., 0., yaw));
    // set translation
    ret.translation().head<2>() = translation_traj_(time + dt);
    ret.translation().z() = height_traj_(time + dt)[0];

    //  ROS_DEBUG_STREAM("Compute feet traj for t = "<<time<<" rotation : : \n"<<
    //                   ret.rotation()<<" \n position : "<<ret.translation().transpose());
    //  ROS_DEBUG_STREAM("Yaw = "<<yaw);
    //  ROS_DEBUG_STREAM("Orientation matrix = \n"<<Spatial::matrixRollPitchYaw(0., 0.,
    //  yaw)); ROS_DEBUG_STREAM("Init rot matrix    = \n"<<pose_init_.rotation());

    return ret;
  }

  pinocchio::SE3 FootTrajectory::start()
  {
    double dt = compute_dt(min());
    return compute(min() + dt);
  }

  pinocchio::SE3 FootTrajectory::end()
  {
    double dt = compute_dt(max());
    return compute(max() + dt);
  }

  eVector3 FootTrajectory::linear_vel(const double& time)
  {
    if (constant_)
    {
      return eVector3::Zero();
    }
    double dt = compute_dt(time);
    eVector3 ret;
    ret.head<2>() = translation_traj_.derivate(time + dt, 1);
    ret.z() = height_traj_.derivate(time + dt, 1)[0];
    return ret;
  }

  eVector3 FootTrajectory::angular_vel(const double& time)
  {
    if (constant_)
    {
      return eVector3::Zero();
    }
    double dt = compute_dt(time);
    return eVector3(0., 0., yaw_traj_.derivate(time + dt, 1)[0]);
  }

  eVector3 FootTrajectory::linear_acc(const double& time)
  {
    if (constant_)
    {
      return eVector3::Zero();
    }
    double dt = compute_dt(time);
    eVector3 ret;
    ret.head<2>() = translation_traj_.derivate(time + dt, 2);
    ret.z() = height_traj_.derivate(time + dt, 2)[0];
    return ret;
  }

  eVector3 FootTrajectory::angular_acc(const double& time)
  {
    if (constant_)
    {
      return eVector3::Zero();
    }
    double dt = compute_dt(time);
    return eVector3(0., 0., yaw_traj_.derivate(time + dt, 2)[0]);
  }

  pinocchio::SE3 FootTrajectory::average(const pinocchio::SE3 &a, const pinocchio::SE3 &b)
  {
    pinocchio::SE3 ret;

    // Translation
    ret.translation() = 0.5 * (a.translation() + b.translation());

    // Rotation
    eQuaternion qa(a.rotation());
    eQuaternion qb(b.rotation());
    qa.normalize();
    qb.normalize();
    eQuaternion qmid = qa.slerp(0.5, qb);
    qmid.normalize();
    ret.rotation(qmid.toRotationMatrix());
    return ret;
  }

}  // namespace memmo
