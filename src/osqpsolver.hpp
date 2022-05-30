#pragma once

#include <memory>

#include "EmbeddedMPC.h"
#include "osqp/qdldl_interface.h"
#include "problem.hpp"

class OSQPSolver {
 public:
  void Initialize(const MPCProblem& prob, std::unique_ptr<OSQPWorkspace> p_work);
  void Solve();
  void GetState(mpc_float* x, int k) const;
  void GetInput(mpc_float* u, int k) const;

 private:
  void BuildKKTSystem();
  void OSQPWorkspaceSetup(void* mem, int memsize);

  int nstates_;
  int ninputs_;
  int nhorizon_;
  std::unique_ptr<OSQPWorkspace> p_workspace_;
};