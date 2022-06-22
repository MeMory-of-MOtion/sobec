#ifndef _FLEXIBILITY_COMPENSATION_
#define _FLEXIBILITY_COMPENSATION_

#include <Eigen/Dense>

#include "sobec/fwd.hpp"
#include <deque>

namespace sobec {
typedef Eigen::Array2d eArray2;
typedef Eigen::Matrix3d eMatrixRot;
typedef Eigen::VectorXd eVectorX;
typedef Eigen::Matrix2d eMatrix2;

// enum side { LEFT, RIGHT };

struct FlexSettings {
 public:
  eVector2 left_stiffness = eVector2(15000, 15000);          // (y, x) [Nm/rad]
  eVector2 left_damping = 2 * left_stiffness.cwiseSqrt();    // (y, x) [Nm/rad]
  eVector2 right_stiffness = eVector2(15000, 15000);         // (y, x) [Nm/rad]
  eVector2 right_damping = 2 * right_stiffness.cwiseSqrt();  // (y, x) [Nm/rad]
  Eigen::Array3i left_hip_indices = Eigen::Array3i::Zero();
  Eigen::Array3i right_hip_indices = Eigen::Array3i::Zero();
  double dt = 0.002, MA_duration = 0.01;                     // [s]

  friend std::ostream &operator<<(std::ostream &out, const FlexSettings &obj) {
    out << "FlexSettings:\n";
    out << "    left_stiffness: " << obj.left_stiffness << "\n";
    out << "    left_damping: " << obj.left_damping << "\n";
    out << "    right_stiffness: " << obj.right_stiffness << "\n";
    out << "    left_damping: " << obj.right_damping << "\n";
    out << "    left_hip_indices: " << obj.left_hip_indices << "\n";
    out << "    right_hip_indices: " << obj.right_hip_indices << "\n";
    out << "    MA_duration: " << obj.MA_duration << "\n";
    out << "    dt: " << obj.dt << "\n" << std::endl;
    return out;
  }

  friend bool operator==(const FlexSettings &lhs, const FlexSettings &rhs) {
    bool test = true;
    test &= lhs.left_stiffness == rhs.left_stiffness;
    test &= lhs.left_damping == rhs.left_damping;
    test &= lhs.right_stiffness == rhs.right_stiffness;
    test &= lhs.right_damping == rhs.right_damping;
    test &= lhs.dt == rhs.dt;
    test &= lhs.left_hip_indices.matrix() == rhs.left_hip_indices.matrix();
    test &= lhs.right_hip_indices.matrix() == rhs.right_hip_indices.matrix();
    test &= lhs.MA_duration == rhs.MA_duration;
    return test;
  }
};

class Flex {
 private:
  
  FlexSettings settings_;
  unsigned long MA_samples_;
  const eMatrix2 xy_to_yx = (eMatrix2() << 0, 1, 1, 0).finished();

  // memory pre-allocations:
  std::deque<eArray2> queue_LH_, queue_RH_;
  eVector3 resulting_angles_;
  eVector2 computed_deflection_;
  eArray2 temp_damping_, temp_actuation_, temp_full_torque_, temp_stiff_;
  eArray2 temp_equiv_stiff_, temp_compliance_;
  eArray2 summation_LH_, summation_RH_, average_;
  unsigned long queueSize_;

  // equivalentAngles:
  double qz_;

  // correctDeflections:
  eVector2 leftFlex0_ = eVector2::Zero();
  eVector2 rightFlex0_ = eVector2::Zero();
  eVector2 leftFlex_, rightFlex_;
  eVector2 leftFlexRate_, rightFlexRate_;

  // correctEstimatedDeflections:
  eMatrix2 adaptLeftYawl_, adaptRightYawl_;
  

  // correctHip:
  eMatrixRot rotationA_;
  eMatrixRot rotationB_;
  eMatrixRot rotationC_;
  eMatrixRot rotationD_;
  eMatrixRot rotationE_;
  eVector3 flexRateY_, flexRateX_;
  eVector3 dqZ_, dqX_, dqY_;
  eVector3 legAngularVelocity_;
  eMatrixRot rigidRotC_, rigidRotD_, M_;

  const eVector3 &equivalentAngles(const eMatrixRot &fullRotation);
  void correctHip(const eVector2 &delta, const eVector2 &deltaDot, eVectorX &q,
                  eVectorX &dq, const Eigen::Array3i &hipIndices);

  // const eArray2 &movingAverage(const eArray2 &x, std::deque<eArray2> &queue);

  const eArray2 &movingAverage(const eArray2 &x, 
                               std::deque<eArray2> &queue, 
                               eArray2 &summation);
 public:
  Flex();

  Flex(const FlexSettings &settings);

  void initialize(const FlexSettings &settings);

  const eVector2 &computeDeflection(const eArray2 &torques, const eArray2 &delta0,
                                    const eArray2 &stiffness, const eArray2 &damping,
                                    const double dt);
  
  void correctDeflections(const eVector2 &leftFlexingTorque,
                          const eVector2 &rightFlexingTorque, eVectorX &q,
                          eVectorX &dq);
                          
  void correctEstimatedDeflections(const eVectorX &desiredTorque, eVectorX &q,
                                   eVectorX &dq);

  const FlexSettings &getSettings() { return settings_; }

  void resetLeftFlex0() { leftFlex0_ = eVector2::Zero(); }// is it used?
  void resetRightFlex0() { rightFlex0_ = eVector2::Zero(); }// is it used?

  void setLeftFlex0(const eVector2 &delta0) { leftFlex0_ = delta0; }// is it used?
  void setRightFlex0(const eVector2 &delta0) { rightFlex0_ = delta0; }// is it used?
};
}  // namespace sobec

#endif
