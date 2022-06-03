#pragma once

#include "EmbeddedMPC.h"
#include "osqp/qdldl_interface.h"
#include "problem.hpp"

class OSQPSolver {
 public:
  OSQPSolver(int nstates, int ninputs, int nhorizon);
  void Initialize(OSQPWorkspace* p_work);
  bool Solve();
  void GetState(mpc_float* x, int k) const;
  void GetInput(mpc_float* u, int k) const;
  void SetInitialState(const mpc_float* x0);
  void SetReferenceState(const mpc_float* xr);
  MPCProblem& GetProblem();
  const c_float* GetSolution() const { return p_workspace_->solution->x; }
  bool GetControl(mpc_float* u, const mpc_float* dx, mpc_float t);

 private:
  void BuildKKTSystem();
  void OSQPWorkspaceSetup(void* mem, int memsize);

  int nstates_;
  int ninputs_;
  int nhorizon_;
  MPCProblem prob_;
  OSQPWorkspace* p_workspace_;
};