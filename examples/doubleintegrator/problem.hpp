#include <ArduinoEigen.h>

#include "problemdata.hpp"

using mpc_float = float;

using StateVector = Eigen::Vector<float, kNumStates>;
using ErrorVector = Eigen::Vector<float, kNumErrStates>;
using InputVector = Eigen::Vector<float, kNumInputs>;
using StateMatrix = Eigen::Matrix<float, kNumStates, kNumStates>;
using InputMatrix = Eigen::Matrix<float, kNumStates, kNumInputs>;
using FeedbackGain = Eigen::Matrix<float, kNumInputs, kNumErrStates>;

class MPCProblem {
public:
  MPCProblem(int nstates, int ninputs, int nhorizon);
  void SetCostTerminalCost(const mpc_float* Qdata, const mpc_float* qdata);
  void SetCostStateCost(const mpc_float* Qdata, const mpc_float* qdata);
  void SetCostInputCost(const mpc_float* Qdata, const mpc_float* qdata);
  void SetCostInputMatrixDiagonal(const mpc_float* Qdata);
  void SetCostInputVector(const mpc_float* qdata);
  void SetCostConstant(mpc_float);
  void SetDynamics(const mpc_float* Adata, const mpc_float* Bdata, const mpc_float* fdata);
  void SetInitialState(const mpc_float* x0);

  int NumStates() const;
  int NumInputs() const;
  int HorizonLength() const;
  void GetCostTerminalCost(mpc_float* Qdata, mpc_float* qdata) const;
  void GetCostStateCost(mpc_float* Qdata, mpc_float* qdata) const;
  void GetCostInputCost(mpc_float* Qdata, mpc_float* qdata) const;
  void GetCostInputMatrixDiagonal(mpc_float* Qdata) const;
  void GetCostInputVector(mpc_float* qdata) const;
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

// void DiscreteDoubleIntegratorDynamics(double h, double dim, Matrix* A, Matrix* B);
