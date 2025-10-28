/**
 * @file transform.hpp
 * @brief Class-based 3D Transformation Library (Header-Only)
 * @author R.Ohnishi
 * @date 2025/10/13
 *
 * @details
 * Inspired by Python's scipy.spatial.transform.Rotation, this library provides
 * a class-based interface for handling 3D rotations and affine transformations.
 *
 * - The `Rotation` class encapsulates a rotation and can apply it to vectors.
 * - The `AffineTransform` class handles combined rotation, translation, and scaling.
 *
 * This is a header-only library. Simply include this file.
 ***************************************************************/
#ifndef _TRANSFORM_HPP
#define _TRANSFORM_HPP

#include <Eigen/Dense>     // For basic vector and matrix operations
#include <Eigen/Geometry>  // For quaternions and AngleAxis

#include <numbers>

/**
 * @brief A namespace for 3D transformation utilities.
 */
namespace Transform3D
{

using Eigen::Matrix3d;
using Eigen::Matrix4d;  // 4x4 affine transformation matrix
using Eigen::Quaterniond;
using Eigen::Vector3d;

class Rotation;         // Forward declaration
class AffineTransform;  // Forward declaration

inline double deg2rad(double deg) { return deg * std::numbers::pi / 180; }
inline double rad2rad(double rad) { return rad * 180 / std::numbers::pi; }

/**
 * @class Rotation
 * @brief Represents a 3D rotation, encapsulating a quaternion.
 *
 * This class provides a clear and robust way to create, compose, and apply
 * 3D rotations. Objects are immutable after creation.
 */
class Rotation
{
public:
  // --- 1. Static Factory Methods for Creation ---

  /**
     * @brief Creates a Rotation from a rotation axis and an angle.
     * @param axis The rotation axis (will be normalized).
     * @param angle_rad The rotation angle in radians.
     * @return Rotation object.
     */
  static Rotation fromAxisAngle(const Vector3d & axis, double angle_rad)
  {
    Eigen::AngleAxisd aa(angle_rad, axis.normalized());
    return Rotation(Quaterniond(aa));
  }

  /**
     * @brief Creates a Rotation from Euler angles (Roll, Pitch, Yaw).
     * Rotation is applied in Z-Y-X order (Yaw, then Pitch, then Roll).
     * @param roll_rad  Rotation about the X-axis in radians.
     * @param pitch_rad Rotation about the Y-axis in radians.
     * @param yaw_rad   Rotation about the Z-axis in radians.
     * @return Rotation object.
     */
  static Rotation fromRPY(double roll_rad, double pitch_rad, double yaw_rad)
  {
    Eigen::AngleAxisd roll(roll_rad, Vector3d::UnitX());
    Eigen::AngleAxisd pitch(pitch_rad, Vector3d::UnitY());
    Eigen::AngleAxisd yaw(yaw_rad, Vector3d::UnitZ());
    return Rotation(yaw * pitch * roll);
  }

  /**
     * @brief Creates a Rotation directly from a quaternion.
     */
  static Rotation fromQuaternion(const Quaterniond & q) { return Rotation(q); }

  /**
     * @brief Creates a Rotation from a 3x3 rotation matrix.
     */
  static Rotation fromRotationMatrix(const Matrix3d & m) { return Rotation(Quaterniond(m)); }

  // --- 2. Constructors ---

  /**
     * @brief Default constructor. Creates an identity rotation (no rotation).
     */
  Rotation() : q_(Quaterniond::Identity()) {}

  // --- 3. Application Methods ---

  /**
     * @brief Applies the rotation to a 3D vector.
     * @param v The vector to rotate.
     * @return The rotated vector.
     */
  Vector3d apply(const Vector3d & v) const { return q_ * v; }

  /**
     * @brief Overload of operator* to apply the rotation to a vector.
     * Enables a natural syntax: `rotated_v = rotation * v;`
     */
  Vector3d operator*(const Vector3d & v) const { return this->apply(v); }

  // --- 4. Composition ---

  /**
     * @brief Composes this rotation with another.
     * `R_combined = R1 * R2` means R2 is applied first, then R1.
     * @param other The rotation to apply before this one.
     * @return The combined Rotation object.
     */
  Rotation operator*(const Rotation & other) const
  {
    // Quaternion multiplication order is the same as rotation order
    return Rotation(this->q_ * other.q_);
  }

  /**
     * @brief Returns the inverse rotation.
     */
  Rotation inverse() const { return Rotation(q_.inverse()); }

  // --- 5. Conversion Methods (Accessors) ---

  /**
     * @brief Returns the internal quaternion representation.
     */
  const Quaterniond & toQuaternion() const { return q_; }

  /**
     * @brief Converts the rotation to a 3x3 rotation matrix.
     */
  Matrix3d toRotationMatrix() const { return q_.toRotationMatrix(); }

  /**
     * @brief Converts the rotation to a 4x4 affine transformation matrix.
     * The translational part is zero.
     */
  Matrix4d toAffineMatrix() const
  {
    Matrix4d mat = Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = this->toRotationMatrix();
    return mat;
  }

private:
  // Store the rotation as a quaternion for efficiency and numerical stability.
  Quaterniond q_;

  // Private constructor to force use of static factory methods.
  explicit Rotation(const Quaterniond & q) : q_(q.normalized()) {}
};

/**
 * @class AffineTransform
 * @brief Represents a 3D affine transformation (rotation, translation, scale).
 */
class AffineTransform
{
public:
  // --- 1. Constructors and Factory Methods ---

  /**
     * @brief Default constructor. Creates an identity transform.
     */
  AffineTransform() : M_(Matrix4d::Identity()) {}

  /**
     * @brief Creates a transform from a 4x4 matrix.
     */
  explicit AffineTransform(const Matrix4d & M) : M_(M) {}

  /**
     * @brief Creates a pure translation transform.
     */
  static AffineTransform fromTranslation(const Vector3d & t)
  {
    Matrix4d mat = Matrix4d::Identity();
    mat.block<3, 1>(0, 3) = t;
    return AffineTransform(mat);
  }

  /**
     * @brief Creates a pure rotation transform.
     */
  static AffineTransform fromRotation(const Rotation & R)
  {
    return AffineTransform(R.toAffineMatrix());
  }

  /**
     * @brief Creates a transform from rotation and translation.
     */
  static AffineTransform fromRotationAndTranslation(const Rotation & R, const Vector3d & t)
  {
    Matrix4d mat = R.toAffineMatrix();
    mat.block<3, 1>(0, 3) = t;
    return AffineTransform(mat);
  }

  // --- 2. Application ---

  /**
     * @brief Applies the transformation to a 3D point.
     * @param v The point to transform.
     * @return The transformed point.
     */
  Vector3d apply(const Vector3d & v) const
  {
    Eigen::Vector4d v_homo(v.x(), v.y(), v.z(), 1.0);
    Eigen::Vector4d v_transformed_homo = M_ * v_homo;
    return v_transformed_homo.head<3>() / v_transformed_homo.w();
  }

  /**
     * @brief Operator overload for applying the transformation.
     */
  Vector3d operator*(const Vector3d & v) const { return this->apply(v); }

  // --- 3. Composition ---

  /**
     * @brief Combines this transform with another.
     * T_combined = T1 * T2 means T2 is applied first, then T1.
     */
  AffineTransform operator*(const AffineTransform & other) const
  {
    return AffineTransform(this->M_ * other.M_);
  }

  /**
     * @brief Returns the inverse transformation.
     */
  AffineTransform inverse() const { return AffineTransform(M_.inverse()); }

  // --- 4. Accessors ---

  /**
     * @brief Returns the underlying 4x4 matrix.
     */
  const Matrix4d & matrix() const { return M_; }

private:
  Matrix4d M_;
};

}  // namespace Transform3D

#endif  // _TRANSFORM_HPP
