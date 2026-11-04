// The MPC drive planner's solve (2026-11-03): pure arithmetic, so the workstation can test this very code against
// sim/lqr/mpc_export.py's Python mirror (FwPlanner). main.cpp (MPC PLANNER) runs it in the planner task and in 'mpcbench'.
#pragma once
#include <math.h>
#include <stdint.h>
#include "mpc_admm.h"       // the QP solver
#include "mpc_plan_data.h"  // GENERATED: the tables (sim/lqr/mpc_export.py)

struct MpcReq { float xi[MPC_X]; float jap, target, vbat; uint8_t held, reset; uint32_t seq; };
// One solve (mpc_export.FwPlanner): xi = Phi xi0 + Gam jap (a step on), the limits l, u = -1 - Y0 xi, 1 - Y0 xi,
// q = Qxi xi + Qt target, ADMM from the last plan shifted a step (reset: from zero). warm (MPC_N): in, the last plan's
// x; out, this one's. J: the jerks (m/s^3). False if anything came out non-finite.
static bool mpcSolve(const MpcReq& r, float* warm, float* J, uint8_t* binOut){
  float m = r.vbat / MPC_VREF; int b = 0;
  for(int i = 1; i < MPC_BINS; i++) if(fabsf(m - MPC_BIN_MOTOR[i]) < fabsf(m - MPC_BIN_MOTOR[b])) b = i;
  const int mode = r.held ? 0 : 1;
  float xi[MPC_X], lo[MPC_M], hi[MPC_M], q[MPC_N], x[MPC_N], z[MPC_M], y[MPC_M], work[2 * MPC_N + MPC_M];
  for(int i = 0; i < MPC_X; i++){
    float s = MPC_GAM[b][i] * r.jap; const float* p = MPC_PHI[b] + i * MPC_X;
    for(int k = 0; k < MPC_X; k++) s += p[k] * r.xi[k];
    xi[i] = s;
  }
  for(int j = 0; j < MPC_M; j++){
    float f = 0.0f; const float* p = MPC_Y0[b] + j * MPC_X;
    for(int k = 0; k < MPC_X; k++) f += p[k] * xi[k];
    lo[j] = -1.0f - f; hi[j] = 1.0f - f;
  }
  for(int i = 0; i < MPC_N; i++){
    float s = MPC_QT[b][mode][i] * r.target; const float* p = MPC_QXI[b][mode] + i * MPC_X;
    for(int k = 0; k < MPC_X; k++) s += p[k] * xi[k];
    q[i] = s;
    x[i] = r.reset ? 0.0f : warm[i < MPC_N - 1 ? i + 1 : i];
  }
  mpcAdmm(MPC_N, MPC_M, MPC_MINV[b][mode], MPC_G[b], q, lo, hi, MPC_RHO, MPC_SIGMA, MPC_ALPHA, MPC_SOFT, MPC_ITERS,
          x, z, y, work);
  bool ok = true;
  for(int i = 0; i < MPC_N; i++){ ok = ok && isfinite(x[i]); warm[i] = x[i]; J[i] = x[i] * MPC_J_SCALE; }
  if(!ok) for(int i = 0; i < MPC_N; i++) warm[i] = 0.0f;
  *binOut = (uint8_t)b;
  return ok;
}

