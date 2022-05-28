#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "EmbeddedMPC.h"
#include "doubleintegrator.h"

TEST(ProblemTests, Constructor) {
  int nhorizon = 3;
  MPCProblem prob(kNumStates, kNumInputs, nhorizon);
  EXPECT_EQ(prob.NumStates(), kNumStates);
  EXPECT_EQ(prob.NumInputs(), kNumInputs);
  EXPECT_EQ(prob.HorizonLength(), nhorizon);
}

TEST(ProblemTests, SetDynamics) {
  int nhorizon = 3;
  MPCProblem prob(kNumStates, kNumInputs, nhorizon);
  prob.SetDynamics(Adata, Bdata, fdata);
  Eigen::MatrixXf A(kNumStates, kNumStates);
  Eigen::MatrixXf B(kNumStates, kNumInputs);
  Eigen::VectorXf f(kNumStates);

  prob.GetDynamics(A.data(), B.data(), f.data());

  Eigen::MatrixXf Atrue(kNumStates, kNumStates);
  Eigen::MatrixXf Btrue(kNumStates, kNumInputs);
  Eigen::VectorXf ftrue(kNumStates);
  // clang-format off
  Atrue << 1,0,h,0,
           0,1,0,h,
           0,0,1,0,
           0,0,0,1;
  Btrue << b,0,
           0,b,
           h,0,
           0,h;
  // clang-format on
  ftrue.setZero();
  EXPECT_TRUE(A.isApprox(Atrue));
  EXPECT_TRUE(B.isApprox(Btrue));
  EXPECT_TRUE(f.isApprox(ftrue));
}
