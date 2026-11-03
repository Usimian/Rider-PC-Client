// The MPC drive planner's QP solver (2026-11-03): sim/lqr/mpc_plan.py's Qp in single precision -- the ESP32's FPU
// does single precision; double is emulated in software.
//
//   min 1/2 x'Px + q'x,  l <= Ax <= u  (soft: an exact penalty, soft x the distance outside a limit -- never
//   infeasible), by ADMM (OSQP's iteration) with a fixed rho.
//
// Minv = (P + sigma I + rho A'A)^-1 is computed offline (P and A do not change with the robot's state), so an iteration
// is matrix-vector work only: ~2 n^2 + 4 m n multiply-adds. Arrays row-major: A is m x n, Minv n x n.
// x: in, the warm start; out, the solution. z, y (m): ADMM's split variable and multiplier, set here.
// work: 2 n + m floats.
#pragma once
#include <math.h>

static void mpcAdmm(int n, int m, const float* Minv, const float* A, const float* q, const float* l, const float* u,
                    float rho, float sigma, float alpha, float soft, int iters,
                    float* x, float* z, float* y, float* work){
  float* rhs = work;            // n
  float* xt  = work + n;        // n
  float* w   = work + 2 * n;    // m: rho z - y
  const float irho = 1.0f / rho, t = soft / rho;
  for(int j = 0; j < m; j++){                                   // z = clip(A x, l, u), y = 0
    const float* a = A + j * n; float s = 0.0f;
    for(int i = 0; i < n; i++) s += a[i] * x[i];
    z[j] = s < l[j] ? l[j] : (s > u[j] ? u[j] : s);
    y[j] = 0.0f;
  }
  for(int it = 0; it < iters; it++){
    for(int j = 0; j < m; j++) w[j] = rho * z[j] - y[j];
    for(int i = 0; i < n; i++) rhs[i] = sigma * x[i] - q[i];
    for(int j = 0; j < m; j++){                                 // rhs += A' w
      const float* a = A + j * n; const float wj = w[j];
      for(int i = 0; i < n; i++) rhs[i] += a[i] * wj;
    }
    for(int i = 0; i < n; i++){                                 // xt = Minv rhs
      const float* r = Minv + i * n; float s = 0.0f;
      for(int k = 0; k < n; k++) s += r[k] * rhs[k];
      xt[i] = s;
    }
    for(int i = 0; i < n; i++) x[i] = alpha * xt[i] + (1.0f - alpha) * x[i];
    for(int j = 0; j < m; j++){                                 // A xt; relaxed; projected (soft); y step
      const float* a = A + j * n; float s = 0.0f;
      for(int i = 0; i < n; i++) s += a[i] * xt[i];
      const float zr = alpha * s + (1.0f - alpha) * z[j];
      const float v = zr + y[j] * irho;
      float zn;
      if(soft > 0.0f) zn = v > u[j] + t ? v - t : (v < l[j] - t ? v + t : (v < l[j] ? l[j] : (v > u[j] ? u[j] : v)));
      else            zn = v < l[j] ? l[j] : (v > u[j] ? u[j] : v);
      y[j] += rho * (zr - zn);
      z[j] = zn;
    }
  }
}
