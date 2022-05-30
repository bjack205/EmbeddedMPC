#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "EmbeddedMPC.h"
#include "osqpsolver.hpp"
#include "doubleintegrator.h"
#include "memory.cpp"

TEST(OSQPSolverTests, Initialization) {
  // Set up a double integrator problem
  int nhorizon = 3;
  MPCProblem prob(kNumStates, kNumInputs, nhorizon);
  // prob.SetDynamics(Adata, Bdata, fdata);
  // prob.SetCostTerminal(Qfdata, qfdata);
  // prob.SetCostState(Qdata, qdata);
  // prob.SetCostInput(Rdata, rdata);
  prob.SetCostConstant(0.0);

  // Initialize OSQPSolver
  OSQPSolver solver;
  constexpr int memory_buffer_size = 1000;
  char buf[memory_buffer_size];
  auto work = std::make_unique<OSQPWorkspace>();

  solver.Initialize(prob, std::move(work));
}
