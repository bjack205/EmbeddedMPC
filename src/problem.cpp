#include "problem.hpp"

MPCProblem::MPCProblem(int nstates, int ninputs, int nhorizon)
    : nstates_(nstates),
      ninputs_(ninputs),
      nhorizon_(nhorizon),
      A_(StateMatrix::Zero(nstates, nstates)),
      B_(InputMatrix::Zero(nstates, ninputs)),
      f_(StateVector::Zero(nstates)),
      x0_(StateVector::Zero(nstates)),
      Qfdiag_(StateVector::Zero(nstates)),
      qf_(StateVector::Zero(nstates)),
      Qdiag_(StateVector::Zero(nstates)),
      q_(StateVector::Zero(nstates)),
      Rdiag_(InputVector::Zero(ninputs)),
      r_(InputVector::Zero(ninputs)),
      c_(0.0) {}

/////////////////////////////////////////////
// Setters
/////////////////////////////////////////////
void MPCProblem::SetCostTerminal(const mpc_float* Qdata, const mpc_float* qdata) {
  memcpy(Qfdiag_.data(), Qdata, sizeof(mpc_float) * nstates_);
  memcpy(qf_.data(), qdata, sizeof(mpc_float) * nstates_);
}
void MPCProblem::SetCostState(const mpc_float* Qdata, const mpc_float* qdata) {
  memcpy(Qdiag_.data(), Qdata, sizeof(mpc_float) * nstates_);
  memcpy(q_.data(), qdata, sizeof(mpc_float) * nstates_);
}
void MPCProblem::SetCostInput(const mpc_float* Rdata, const mpc_float* rdata) {
  memcpy(Rdiag_.data(), Rdata, sizeof(mpc_float) * ninputs_);
  memcpy(r_.data(), rdata, sizeof(mpc_float) * ninputs_);
}
void MPCProblem::SetCostConstant(mpc_float c) { c_ = c; }

void MPCProblem::SetDynamics(const mpc_float* Adata, const mpc_float* Bdata,
                             const mpc_float* fdata) {
  memcpy(A_.data(), Adata, sizeof(mpc_float) * nstates_ * nstates_);
  memcpy(B_.data(), Bdata, sizeof(mpc_float) * nstates_ * ninputs_);
  memcpy(f_.data(), fdata, sizeof(mpc_float) * nstates_);
}
void MPCProblem::SetInitialState(const mpc_float* x0) {
  memcpy(x0_.data(), x0, sizeof(mpc_float) * nstates_);
}

/////////////////////////////////////////////
// Getters
/////////////////////////////////////////////
int MPCProblem::NumStates() const { return nstates_; }
int MPCProblem::NumInputs() const { return ninputs_; }
int MPCProblem::HorizonLength() const { return nhorizon_; }

void MPCProblem::GetCostTerminal(mpc_float* Qdata, mpc_float* qdata) const {
  memcpy(Qdata, Qfdiag_.data(), sizeof(mpc_float) * nstates_);
  memcpy(qdata, qf_.data(), sizeof(mpc_float) * nstates_);
}
void MPCProblem::GetCostState(mpc_float* Qdata, mpc_float* qdata) const {
  memcpy(Qdata, Qdiag_.data(), sizeof(mpc_float) * nstates_);
  memcpy(qdata, q_.data(), sizeof(mpc_float) * nstates_);
}
void MPCProblem::GetCostInput(mpc_float* Rdata, mpc_float* rdata) const {
  memcpy(Rdata, Rdiag_.data(), sizeof(mpc_float) * ninputs_);
  memcpy(rdata, r_.data(), sizeof(mpc_float) * ninputs_);
}
mpc_float MPCProblem::GetCostConstant() const { return c_; }

void MPCProblem::GetDynamics(mpc_float* Adata, mpc_float* Bdata, mpc_float* fdata) const {
  memcpy(Adata, A_.data(), sizeof(mpc_float) * nstates_ * nstates_);
  memcpy(Bdata, B_.data(), sizeof(mpc_float) * nstates_ * ninputs_);
  memcpy(fdata, f_.data(), sizeof(mpc_float) * nstates_);
}
void MPCProblem::GetInitialState(mpc_float* x0) const {
  memcpy(x0, x0_.data(), sizeof(mpc_float) * nstates_);
}