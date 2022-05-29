#include "osqpsolver.hpp"

#include "memory.hpp"

void OSQPSolver::Initialize(const MPCProblem& prob, void* mem, int memsize) {
  int n = prob.NumStates();
  int m = prob.NumInputs();
  int N = prob.HorizonLength();
  int nvars = N * n + (N - 1) * m;
  int ncon_dynamics = N * n;  // includes initial condition
  char* p_mem = (char*)mem;

  // P matrix
  int P_nrows = nvars;
  int P_ncols = nvars;
  int P_nnz = nvars;
  c_int* Pdata_i = AllocateBuffer<c_int>(&p_mem, P_nnz);
  c_int* Pdata_p = AllocateBuffer<c_int>(&p_mem, P_nrows + 1);
  c_float* Pdata_x = AllocateBuffer<c_float>(&p_mem, P_nnz);
  Pdata = {P_nnz, P_nrows, P_ncols, Pdata_p, Pdata_i, Pdata_x, -1};

  // A matrix
  int A_nrows = ncon_dynamics;
  int A_ncols = nvars;
  int A_nnz_identities = N * n;  // includes initial condition
  int A_nnz_state = (N - 1) * n * n;
  int A_nnz_input = (N - 1) * n * m;
  int A_nnz = A_nnz_state + A_nnz_input + A_nnz_identities;
  c_int* Adata_i = AllocateBuffer<c_int>(&p_mem, A_nnz);
  c_int* Adata_p = AllocateBuffer<c_int>(&p_mem, A_nrows + 1);
  c_float* Adata_x = AllocateBuffer<c_float>(&p_mem, A_nnz);
  Adata = {A_nnz, A_nrows, A_ncols, Adata_p, Adata_i, Adata_x, -1};

  // vectors
  qdata = AllocateBuffer<c_float>(&p_mem, nvars);
  ldata = AllocateBuffer<c_float>(&p_mem, ncon_dynamics);
  udata = AllocateBuffer<c_float>(&p_mem, ncon_dynamics);

  // OSQP Data
  int nprimals = nvars;
  int nduals = ncon_dynamics;
  // data = {nprimals, nduals, &Pdata, &Adata, qdata, ldata, udata};

  // Scaling
  Dscaling = AllocateBuffer<c_float>(&p_mem, nprimals);
  Dinvscaling = AllocateBuffer<c_float>(&p_mem, nprimals);
  Escaling = AllocateBuffer<c_float>(&p_mem, nprimals);
  Einvscaling = AllocateBuffer<c_float>(&p_mem, nprimals);
  float cost_scaling = 1.0;  // TODO: use a better scaling
  float cost_scaling_inv = 1.0 / cost_scaling;
  scaling = {cost_scaling, Dscaling, Escaling, cost_scaling_inv, Dinvscaling, Einvscaling};

  // QDLDL Initialization
  // c_float* linsys_solver_Dinv;         // (nprimal+ndual,)
  // c_int* linsys_solver_P;              // (nprimal+ndual,)
  // c_float* linsys_solver_bp;           // (nprimal+ndual,)
  // c_float* linsys_solver_sol;          // (nprimal+ndual,)
  // c_float* linsys_solver_rho_inv_vec;  // (ndual,)
  // c_int* linsys_solver_Pdiag_idx;      // (nprimal,)
  // csc linsys_solver_KKT;
  // c_int* linsys_solver_PtoKKT;         // (nprimal,)
  // c_int* linsys_solver_AtoKKT;         // (nnz?,)
  // c_int* linsys_solver_rhotoKKT;       // (ndual,)
  // QDLDL_float* linsys_solver_D;        // (nprimal+ndual,)
  // QDLDL_int* linsys_solver_etree;      // (nprimal+ndual,)
  // QDLDL_int* linsys_solver_Lnz;        // (nprimal+ndual,)
  // QDLDL_int* linsys_solver_iwork;      //
  // QDLDL_bool* linsys_solver_bwork;     // (nprimal_ndual,)
  // QDLDL_float* linsys_solver_fwork;    // (nprimal_ndual,)
  // qdldl_solver linsys_solver;

  // Calculate total memory usage
  memsize_ = (int)(p_mem - (char*)mem);
}

int OSQPSolver::GetTotalMemorySize() const { return memsize_; }

void OSQPSolver::OSQPWorkspaceSetup(void* mem, int memsize) {
  c_int exitflag;

  // // Copy problem data into workspace
  // work_ = {&data_,
  //          (LinSysSolver*)&linsys_solver,
  //          work_rho_vec,
  //          work_rho_inv_vec,
  //          work_constr_type,
  //          work_x,
  //          work_y,
  //          work_z,
  //          work_xz_tilde,
  //          work_x_prev,
  //          work_z_prev,
  //          work_Ax,
  //          work_Px,
  //          work_Aty,
  //          work_delta_y,
  //          work_Atdelta_y,
  //          work_delta_x,
  //          work_Pdelta_x,
  //          work_Adelta_x,
  //          work_D_temp,
  //          work_D_temp_A,
  //          work_E_temp,
  //          &settings,
  //          &scaling,
  //          &solution,
  //          &info};
}