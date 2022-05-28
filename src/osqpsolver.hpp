#pragma once

#include "EmbeddedMPC.h"
#include "problem.hpp"

class OSQPSolver {
 public:
  void Initialize(const MPCProblem& prob);
  void Solve();
  void GetState(mpc_float* x, int k);
  void GetInput(mpc_float* u, int k);

 private:
  void BuildKKTSystem();

  // Data structure prototypes
  csc Pdata;
  csc Adata;
  c_float qdata[172];
  c_float ldata[304];
  c_float udata[304];
  OSQPData data;

  // // Settings structure prototype
  // OSQPSettings settings;

  // // Scaling structure prototypes
  // c_float Dscaling[172];
  // c_float Dinvscaling[172];
  // c_float Escaling[304];
  // c_float Einvscaling[304];
  // OSQPScaling scaling;

  // // Prototypes for linsys_solver structure
  // csc linsys_solver_L;
  // c_float linsys_solver_Dinv[476];
  // c_int linsys_solver_P[476];
  // c_float linsys_solver_bp[476];
  // c_float linsys_solver_sol[476];
  // c_float linsys_solver_rho_inv_vec[304];
  // c_int linsys_solver_Pdiag_idx[117];
  // csc linsys_solver_KKT;
  // c_int linsys_solver_PtoKKT[117];
  // c_int linsys_solver_AtoKKT[1044];
  // c_int linsys_solver_rhotoKKT[304];
  // QDLDL_float linsys_solver_D[476];
  // QDLDL_int linsys_solver_etree[476];
  // QDLDL_int linsys_solver_Lnz[476];
  // QDLDL_int linsys_solver_iwork[1428];
  // QDLDL_bool linsys_solver_bwork[476];
  // QDLDL_float linsys_solver_fwork[476];
  // qdldl_solver linsys_solver;

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