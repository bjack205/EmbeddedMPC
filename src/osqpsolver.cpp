#include "osqpsolver.hpp"

#include "memory.hpp"

void OSQPSolver::Initialize(const MPCProblem& prob, std::unique_ptr<OSQPWorkspace> p_work) {
  int n = prob.NumStates();
  int m = prob.NumInputs();
  int N = prob.HorizonLength();
  int nvars = N * n + (N - 1) * m;
  int ncon_dynamics = N * n;  // includes initial condition

  // Copy data to solver
  nstates_ = n;
  ninputs_ = m;
  nhorizon_ = N;

  // Move workspace to class
  p_workspace_ = std::move(p_work);
}

void OSQPSolver::Solve() {
  osqp_solve(p_workspace_.get());
}

void OSQPSolver::GetState(mpc_float* x, int k) const {
  const c_float* sol = p_workspace_->solution->x;
  for (int i = 0; i < nstates_; ++i) {
    x[i] = sol[i + k * nstates_];
  }
}

void OSQPSolver::GetInput(mpc_float* u, int k) const {
  const c_float* sol = p_workspace_->solution->x;
  const int num_states_total = nhorizon_ * nstates_;
  for (int i = 0; i < nstates_; ++i) {
    u[i] = sol[i + num_states_total + k * ninputs_];
  }
}