#include "osqpsolver.hpp"

#include "memory.hpp"

void OSQPSolver::Initialize(const MPCProblem& prob, std::unique_ptr<OSQPWorkspace> p_work) {
  int n = prob.NumStates();
  int m = prob.NumInputs();
  int N = prob.HorizonLength();
  int nvars = N * n + (N - 1) * m;
  int ncon_dynamics = N * n;  // includes initial condition
}
