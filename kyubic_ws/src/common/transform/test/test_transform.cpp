#include <gtest/gtest.h>

#include <cmath>
#include <transform/transform.hpp>  // 自身のパッケージのヘッダーをインクルード

// isApprox()を使うために必要
using namespace Eigen;

// テスト用の許容誤差
constexpr double TOLERANCE = 1e-9;

// ===== 1. Rotation Class Tests =====

// TEST(TestSuiteName, TestCaseName)
TEST(RotationTest, DefaultConstructorIsIdentity)
{
  using namespace Transform3D;
  Rotation r;
  Vector3d v(1.0, 2.0, 3.0);
  Vector3d v_rotated = r * v;

  // isApprox() でベクトルがほぼ等しいか比較
  EXPECT_TRUE(v_rotated.isApprox(v, TOLERANCE));
  EXPECT_TRUE(r.toQuaternion().isApprox(Quaterniond::Identity(), TOLERANCE));
}

TEST(RotationTest, FromRpyKnownRotation)
{
  using namespace Transform3D;
  // Z軸周りに90度回転 (ヨー)
  Rotation r_yaw90 = Rotation::fromRPY(0, 0, M_PI / 2.0);

  Vector3d v_x(1.0, 0.0, 0.0);
  Vector3d v_y(0.0, 1.0, 0.0);

  Vector3d v_rotated = r_yaw90 * v_x;

  // X軸ベクトル(1,0,0)をZ軸周りに90度回すとY軸ベクトル(0,1,0)になる
  EXPECT_TRUE(v_rotated.isApprox(v_y, TOLERANCE));
}

TEST(RotationTest, FromAxisAngleKnownRotation)
{
  using namespace Transform3D;
  // Y軸周りに90度回転
  Rotation r = Rotation::fromAxisAngle(Vector3d::UnitY(), M_PI / 2.0);

  Vector3d v_x(1.0, 0.0, 0.0);
  Vector3d v_z_neg(0.0, 0.0, -1.0);

  Vector3d v_rotated = r * v_x;

  // X軸ベクトル(1,0,0)をY軸周りに-90度回すと-Z軸ベクトル(0,0,-1)になる
  EXPECT_TRUE(v_rotated.isApprox(v_z_neg, TOLERANCE));
}

TEST(RotationTest, Composition)
{
  using namespace Transform3D;
  // X軸90度回転
  Rotation r_x90 = Rotation::fromRPY(M_PI / 2.0, 0, 0);
  // Y軸90度回転
  Rotation r_y90 = Rotation::fromRPY(0, M_PI / 2.0, 0);

  // 合成: 最初にr_y90を適用し、次にr_z90を適用
  Rotation r_combined = r_x90 * r_y90;

  Vector3d v_x(1.0, 0.0, 0.0);
  // 期待値: (1,0,0) --(Y90度)--> (0,0,-1) --(X90度)--> (0,1,0)
  Vector3d expected_v(0.0, 1.0, 0.0);

  Vector3d v_rotated = r_combined * v_x;
#include <iostream>
  std::cout << v_rotated << std::endl;
  EXPECT_TRUE(v_rotated.isApprox(expected_v, TOLERANCE));
}

TEST(RotationTest, Inverse)
{
  using namespace Transform3D;
  Rotation r = Rotation::fromRPY(0.1, 0.2, 0.3);
  Rotation r_inv = r.inverse();

  Vector3d v(1.0, 2.0, 3.0);

  // 回転を適用し、その逆回転を適用すると元のベクトルに戻る
  Vector3d v_back = r_inv * (r * v);

  EXPECT_TRUE(v_back.isApprox(v, TOLERANCE));
}

// ===== 2. AffineTransform Class Tests =====

TEST(AffineTransformTest, DefaultConstructorIsIdentity)
{
  using namespace Transform3D;
  AffineTransform T;
  Vector3d p(4.0, 5.0, 6.0);
  Vector3d p_transformed = T * p;

  EXPECT_TRUE(p_transformed.isApprox(p, TOLERANCE));
  EXPECT_TRUE(T.matrix().isApprox(Matrix4d::Identity(), TOLERANCE));
}

TEST(AffineTransformTest, FromTranslation)
{
  using namespace Transform3D;
  Vector3d t(10.0, -20.0, 30.0);
  AffineTransform T = AffineTransform::fromTranslation(t);

  Vector3d p(1.0, 2.0, 3.0);
  Vector3d expected_p(11.0, -18.0, 33.0);

  Vector3d p_transformed = T * p;
  EXPECT_TRUE(p_transformed.isApprox(expected_p, TOLERANCE));
}

TEST(AffineTransformTest, FromRotationAndTranslation)
{
  using namespace Transform3D;
  // Z軸90度回転
  Rotation R = Rotation::fromRPY(0, 0, M_PI / 2.0);
  // (10, 0, 0) の並進
  Vector3d t(10.0, 0.0, 0.0);

  AffineTransform T = AffineTransform::fromRotationAndTranslation(R, t);

  Vector3d p(1.0, 0.0, 0.0);
  // 期待値: p(1,0,0) --(回転)--> (0,1,0) --(並進)--> (10,1,0)
  Vector3d expected_p(10.0, 1.0, 0.0);

  Vector3d p_transformed = T * p;
  EXPECT_TRUE(p_transformed.isApprox(expected_p, TOLERANCE));
}

TEST(AffineTransformTest, Composition)
{
  using namespace Transform3D;
  // T1: Z軸90度回転
  AffineTransform T1 = AffineTransform::fromRotation(Rotation::fromRPY(0, 0, M_PI / 2.0));
  // T2: X方向に+5並進
  AffineTransform T2 = AffineTransform::fromTranslation(Vector3d(5.0, 0.0, 0.0));

  // 合成: 最初にT2(並進)を適用し、次にT1(回転)を適用
  AffineTransform T_combined = T1 * T2;

  Vector3d p(1.0, 0.0, 0.0);
  // 期待値: p(1,0,0) --(T2:並進)--> (6,0,0) --(T1:回転)--> (0,6,0)
  Vector3d expected_p(0.0, 6.0, 0.0);

  Vector3d p_transformed = T_combined * p;
  EXPECT_TRUE(p_transformed.isApprox(expected_p, TOLERANCE));
}
