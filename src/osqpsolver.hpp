#pragma once

#include "EmbeddedMPC.h"
#include "osqp/qdldl_interface.h"
#include "problem.hpp"

class OSQPSolver {
 public:
  void Initialize(const MPCProblem& prob, void* mem, int memsize);
  void Solve();
  void GetState(mpc_float* x, int k);
  void GetInput(mpc_float* u, int k);
  int GetTotalMemorySize() const;

 private:
  void BuildKKTSystem();
  void OSQPWorkspaceSetup(void* mem, int memsize);
  int memsize_;

  OSQPWorkspace work_;
  OSQPData data_;

  // Data structure prototypes
  csc Pdata;
  csc Adata;
  c_float* qdata;
  c_float* ldata;
  c_float* udata;

  // Settings structure prototype
  OSQPSettings settings;

  // Scaling structure prototypes
  c_float* Dscaling;     ///< primal scaling (nprimal,)
  c_float* Dinvscaling;  //                  (nprimal,)
  c_float* Escaling;     ///< dual scaling   (ndual,)
  c_float* Einvscaling;  //                  (ndual,)
  OSQPScaling scaling;

  // Prototypes for linsys_solver structure
  csc linsys_solver_L;
  c_float* linsys_solver_Dinv;         // (nprimal+ndual,)
  c_int* linsys_solver_P;              // (nprimal+ndual,)
  c_float* linsys_solver_bp;           // (nprimal+ndual,)
  c_float* linsys_solver_sol;          // (nprimal+ndual,)
  c_float* linsys_solver_rho_inv_vec;  // (ndual,)
  c_int* linsys_solver_Pdiag_idx;      // (nprimal,)
  csc linsys_solver_KKT;
  c_int* linsys_solver_PtoKKT;       // (nprimal,)
  c_int* linsys_solver_AtoKKT;       // (nnz?,)
  c_int* linsys_solver_rhotoKKT;     // (ndual,)
  QDLDL_float* linsys_solver_D;      // (nprimal+ndual,)
  QDLDL_int* linsys_solver_etree;    // (nprimal+ndual,)
  QDLDL_int* linsys_solver_Lnz;      // (nprimal+ndual,)
  QDLDL_int* linsys_solver_iwork;    //
  QDLDL_bool* linsys_solver_bwork;   // (nprimal_ndual,)
  QDLDL_float* linsys_solver_fwork;  // (nprimal_ndual,)
  qdldl_solver linsys_solver;

  // // Prototypes for solution
  // c_float xsolution[172];
  // c_float ysolution[304];

  // OSQPSolution solution;

  // // Prototype for info structure
  // OSQPInfo info;

  // // Prototypes for the workspace
  // c_float work_rho_vec[304];
  // c_float work_rho_inv_vec[304];
  // c_int work_constr_type[304];
  // c_float work_x[172];
  // c_float work_y[304];
  // c_float work_z[304];
  // c_float work_xz_tilde[476];
  // c_float work_x_prev[172];
  // c_float work_z_prev[304];
  // c_float work_Ax[304];
  // c_float work_Px[172];
  // c_float work_Aty[172];
  // c_float work_delta_y[304];
  // c_float work_Atdelta_y[172];
  // c_float work_delta_x[172];
  // c_float work_Pdelta_x[172];
  // c_float work_Adelta_x[304];
  // c_float work_D_temp[172];
  // c_float work_D_temp_A[172];
  // c_float work_E_temp[304];

  // OSQPWorkspace workspace;
};