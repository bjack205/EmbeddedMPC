#include "problem.hpp"

MPCProblem::MPCProblem(int nstates, int ninputs, int nhorizon)
    : nstates_(nstates), ninputs_(ninputs), nhorizon_(nhorizon), c_(0.0) {
  A_.setZero();
  B_.setZero();
  f_.setZero();
  x0_.setZero();
  Qfdiag_.setZero();
  qf_.setZero();
  Qdiag_.setZero();
  q_.setZero();
  Rdiag_.setZero();
  r_.setZero();
}
