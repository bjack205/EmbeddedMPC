#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "EmbeddedMPC.h"
#include "problem.hpp"
// #include "doubleintegrator.h"
#include "problem_data.h"

const float h = 0.01;
const float b = h * h / 2.0;

TEST(ProblemTests, Constructor) {
  int nhorizon = 3;
  MPCProblem prob(nstates, ninputs, nhorizon);
  EXPECT_EQ(prob.NumStates(), nstates);
  EXPECT_EQ(prob.NumInputs(), ninputs);
  EXPECT_EQ(prob.HorizonLength(), nhorizon);
}

// TEST(ProblemTests, Dynamics) {
//   int nhorizon = 3;
//   MPCProblem prob(nstates, ninputs, nhorizon);
//   prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);

//   Eigen::MatrixXd A(nstates, nstates);
//   Eigen::MatrixXd B(nstates, ninputs);
//   Eigen::VectorXd f(nstates);
//   prob.GetDynamics(A.data(), B.data(), f.data());

//   Eigen::MatrixXd Atrue(nstates, nstates);
//   Eigen::MatrixXd Btrue(nstates, ninputs);
//   Eigen::VectorXd ftrue(nstates);
//   // clang-format off
//   Atrue << 1,0,h,0,
//            0,1,0,h,
//            0,0,1,0,
//            0,0,0,1;
//   Btrue << b,0,
//            0,b,
//            h,0,
//            0,h;
//   // clang-format on
//   ftrue.setZero();
//   EXPECT_TRUE(A.isApprox(Atrue));
//   EXPECT_TRUE(B.isApprox(Btrue));
//   EXPECT_TRUE(f.isApprox(ftrue));
// }
