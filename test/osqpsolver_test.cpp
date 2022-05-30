#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "EmbeddedMPC.h"
#include "osqpsolver.hpp"
// #include "doubleintegrator.h"
#include "workspace.h"
#include "problem_data.h"
#include "memory.cpp"

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
  fmt::print("x0 = {}\n", x0);
  fmt::print("u0 = {}\n", u0);
}
