#include "osqpsolver.hpp"

#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "EmbeddedMPC.h"
// #include "doubleintegrator.h"
#include "memory.cpp"
#include "problem_data.h"
#include "workspace.h"

TEST(OSQPSolverTests, RawSolve) {
  osqp_solve(&workspace);
  // Print status
  printf("Status:                %s\n", (&workspace)->info->status);
  printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  printf("Primal residual:       %.4e\n", (&workspace)->info->pri_res);
  printf("Dual residual:         %.4e\n", (&workspace)->info->dua_res);
  c_float* x = workspace.solution->x;
  printf("x0 = [ ");
  for (int i = 0; i < 12; ++i) {
    printf("%0.3g ", x[i]);
  }
  printf("]\n");

  int nstates_total = 12 * 11;
  printf("u0 = [ ");
  for (int i = 0; i < 4; ++i) {
    printf("%0.4g ", x[i + nstates_total]);
  }
  printf("]\n");
}

TEST(OSQPSolverTests, Initialization) {
  // Set up a double integrator problem
  int nhorizon = 3;

  // Initialize OSQPSolver
  OSQPSolver solver(nstates, ninputs, nhorizon);
  MPCProblem& prob = solver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(0.0);

  solver.Initialize(&workspace);
  c_float x0[12];
  solver.GetState(x0, 0);
}

TEST(OSQPSolverTests, Solve) {
  OSQPSolver solver(nstates, ninputs, nhorizon);
  MPCProblem& prob = solver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(0.0);

  solver.Initialize(&workspace);
  solver.Solve();
  Eigen::Vector<c_float, 12> x0;
  Eigen::Vector<c_float, 4> u0;
  solver.GetState(x0.data(), 0);
  solver.GetInput(u0.data(), 0);
  Eigen::Vector<c_float, 4> u_expected;
  // u_expected << 12.02562626, 12.02562626, 12.02562626, 12.02562626;
  u_expected << 55.43705474, 55.43705474, 55.43705474, 55.43705474;
  EXPECT_LT(x0.norm(), 1e-5);
  EXPECT_LT((u0-u_expected).norm(), 1e-5);
  EXPECT_STREQ(workspace.info->status, "solved");
}

TEST(OSQPSolverTests, GetControl) {
  OSQPSolver solver(nstates, ninputs, nhorizon);
  MPCProblem& prob = solver.GetProblem();
  prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
  prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
  prob.SetCostState(cost_Qdata, cost_qdata);
  prob.SetCostInput(cost_Rdata, cost_rdata);
  prob.SetCostConstant(0.0);
  prob.SetEquilibriumPoint(dynamics_xe, dynamics_ue);
  prob.SetGoalState(dynamics_xg);

  solver.Initialize(&workspace);
  Eigen::Vector<mpc_float,13> xe = Eigen::Vector<mpc_float, 13>::Zero();
  Eigen::Vector<mpc_float,4> ue = Eigen::Vector<mpc_float, 4>::Zero();
  Eigen::Vector<mpc_float,13> x0 = Eigen::Vector<mpc_float, 13>::Zero();
  Eigen::Vector<mpc_float,13> xg = Eigen::Vector<mpc_float, 13>::Zero();
  prob.GetEquilibriumPoint(xe.data(), ue.data());
  prob.GetGoalState(xg.data());
  prob.GetInitialState(x0.data());
  // solver.GetControl()
  // solver.Solve();
  // Eigen::Vector<c_float, 12> x0;
  // Eigen::Vector<c_float, 4> u0;
  // solver.GetState(x0.data(), 0);
  // solver.GetInput(u0.data(), 0);
  // Eigen::Vector<c_float, 4> u_expected;
  // // u_expected << 12.02562626, 12.02562626, 12.02562626, 12.02562626;
  // u_expected << 55.43705474, 55.43705474, 55.43705474, 55.43705474;
  // EXPECT_LT(x0.norm(), 1e-5);
  // EXPECT_LT((u0-u_expected).norm(), 1e-5);
  // EXPECT_STREQ(workspace.info->status, "solved");
}