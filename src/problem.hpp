#pragma once

#include <ArduinoEigen/Eigen/Dense>

// #include "problemdata.hpp"

using mpc_float = float;
using mpc_int = int;

using StateVector = Eigen::Vector<float, Eigen::Dynamic>;
using ErrorVector = Eigen::Vector<float, Eigen::Dynamic>;
using InputVector = Eigen::Vector<float, Eigen::Dynamic>;
using StateMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using InputMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using FeedbackGain = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;

class MPCProblem {
 public:
  MPCProblem(int nstates, int ninputs, int nhorizon);
  void SetCostTerminal(const mpc_float* Qdata, const mpc_float* qdata);
  void SetCostState(const mpc_float* Qdata, const mpc_float* qdata);
  void SetCostInput(const mpc_float* Rdata, const mpc_float* rdata);
  void SetCostConstant(mpc_float c);
  void SetDynamics(const mpc_float* Adata, const mpc_float* Bdata, const mpc_float* fdata);
  void SetInitialState(const mpc_float* x0);

  int NumStates() const;
  int NumInputs() const;
  int HorizonLength() const;
  void GetCostTerminal(mpc_float* Qdata, mpc_float* qdata) const;
  void GetCostState(mpc_float* Qdata, mpc_float* qdata) const;
  void GetCostInput(mpc_float* Rdata, mpc_float* rdata) const;
  mpc_float GetCostConstant() const;
  void GetDynamics(mpc_float* Adata, mpc_float* Bdata, mpc_float* fdata) const;
  void GetInitialState(mpc_float* x0) const;

 private:
  int nstates_;
  int ninputs_;
  int nhorizon_;

  // Dynamics
  StateMatrix A_;
  InputMatrix B_;
  StateVector f_;
  StateVector x0_;

  // Cost
  StateVector Qfdiag_;
  StateVector qf_;
  StateVector Qdiag_;
  StateVector q_;
  InputVector Rdiag_;
  InputVector r_;
  mpc_float c_;
};

// void DiscreteDoubleIntegratorDynamics(double h, double dim, Matrix* A,
// Matrix* B);
