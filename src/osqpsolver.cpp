#include "osqpsolver.hpp"

#include "memory.hpp"
OSQPSolver::OSQPSolver(int nstates, int ninputs, int nhorizon)
    : nstates_(nstates),
      ninputs_(ninputs),
      nhorizon_(nhorizon),
      prob_(nstates_, ninputs_, nhorizon_) {}

void OSQPSolver::Initialize(OSQPWorkspace* p_work) {
  // Move workspace to class
  p_workspace_ = p_work; 
}

bool OSQPSolver::Solve() { 
  osqp_solve(p_workspace_); 
  return p_workspace_->info->status_val == OSQP_SOLVED;
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
  for (int i = 0; i < ninputs_; ++i) {
    u[i] = sol[i + num_states_total + k * ninputs_];
  }
}

void OSQPSolver::SetInitialState(const mpc_float* x0) {
  c_float* l = p_workspace_->data->l;
  c_float* u = p_workspace_->data->u;
  for (int i = 0; i < nstates_; ++i) {
    l[i] = x0[i];
    u[i] = x0[i];
  }
  osqp_update_bounds(p_workspace_, l, u);
}

void OSQPSolver::SetReferenceState(const mpc_float* xr) {
  const mpc_float* Qf = prob_.GetCostDiagonalTerminal();
  const mpc_float* Qk = prob_.GetCostDiagonalState();
  c_float* q = p_workspace_->data->q;
  for (int k = 0; k < nhorizon_; ++k) {
    int offset = k * nstates_;
    const mpc_float* Q = k == nhorizon_ - 1 ? Qf : Qk;
    for (int i = 0; i < nstates_; ++i) {
      q[i + offset] = -Q[i] * xr[i];
    }
  }
  osqp_update_lin_cost(p_workspace_, q);
}

MPCProblem& OSQPSolver::GetProblem() { return prob_; }

bool OSQPSolver::GetControl(mpc_float* u, const mpc_float* dx, mpc_float t) {
  SetInitialState(dx);
  bool solve_successful = Solve();
  GetInput(u, 0);

  // Add equilibrium control
  const int m = prob_.NumInputs();
  const mpc_float* ueq = prob_.GetEquilibriumInput();
  for (int i = 0; i < m; ++i) {
    if (solve_successful) {
      u[i] += ueq[i];
    } else {
      u[i] = ueq[i];
    }
  }
  return solve_successful;
}