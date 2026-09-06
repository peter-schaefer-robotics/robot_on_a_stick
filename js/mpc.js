/*
 * Model predictive control (MPC) in condensed (dense) form for the cart-pole.
 *
 * Optimization problem solved at every control step:
 *
 *   min_U  sum_{k=0}^{N-1} (x_k - x_ref)' Q (x_k - x_ref) + R u_k^2
 *          + (x_N - x_ref)' P (x_N - x_ref)
 *   s.t.   x_{k+1} = Ad_k x_k + Bd_k u_k + cd_k     (prediction model)
 *          |u_k| <= u_max                           (input bound)
 *          x_0 = x(t)                               (current state)
 *
 * Two prediction models are available:
 *
 *   'linear'    A single linearisation about the upright equilibrium. This is
 *               classic linear MPC: Ad, Bd and H are constant and are only
 *               rebuilt when a parameter changes.
 *   'nonlinear' Real-time iteration (SQP with one iteration per step): the
 *               previous solution is simulated forward through the *nonlinear*
 *               model and the system is re-linearised at every point along
 *               that trajectory, giving a linear time-varying model that stays
 *               valid at large deflections.
 *
 * How it is solved:
 *  1) The affine term cd is embedded via the augmented state z = [x; 1] so the
 *     prediction stays purely linear.
 *  2) Eliminating the states gives X = Phi z_0 + Gamma U, leaving a
 *     box-constrained QP in U (N variables):
 *        min_U 1/2 U' H U + g' U + const,   -u_max <= U <= u_max
 *        H = 2 (Gamma' Qbar Gamma + R I),  g = 2 Gamma' Qbar (Phi z_0 - Xref)
 *  3) The unconstrained solution is computed exactly from a Cholesky
 *     factorisation of H. If it lies inside the box it already is the optimum;
 *     otherwise it is clipped and used to start a projected coordinate descent
 *     that identifies the active set.
 *  4) Only u_0 is applied; at the next step everything restarts (receding
 *     horizon).
 */
(function (root) {
  'use strict';

  var IPM = root.IPM = root.IPM || {};
  var LA = IPM.linalg;
  var NX = 4;   // state dimension
  var NZ = 5;   // augmented state z = [x; 1]

  var now = (root.performance && root.performance.now)
    ? function () { return root.performance.now(); }
    : function () { return Date.now(); };

  /**
   * Exact zero-order-hold discretisation of x_dot = A x + B u + c via the
   * matrix exponential of the augmented system
   *   S = [[A, c, B], [0, 0, 0], [0, 0, 0]],   exp(S*Ts).
   * Returns Ad ((n+1)x(n+1)) for z = [x;1] and Bd ((n+1)x1).
   */
  function discretize(A, B, c, Ts) {
    var S = LA.mat(NX + 2, NX + 2);
    var i, j;
    for (i = 0; i < NX; i++) {
      for (j = 0; j < NX; j++) S.d[i * (NX + 2) + j] = A[i][j];
      S.d[i * (NX + 2) + NX] = c[i];
      S.d[i * (NX + 2) + NX + 1] = B[i];
    }
    var E = LA.expm(LA.scale(S, Ts));
    var Ad = LA.block(E, 0, NZ, 0, NZ);
    var Bd = new Float64Array(NZ);
    for (i = 0; i < NZ; i++) Bd[i] = E.d[i * (NX + 2) + NX + 1];
    return { Ad: Ad, Bd: Bd };
  }

  /**
   * Discrete algebraic Riccati equation, solved by value iteration.
   * With a single input, R + B'PB is a scalar, so no linear solver is needed.
   * P is the terminal weight and approximates the remaining cost beyond the
   * horizon (the infinite-horizon LQR).
   */
  function dare(Ad, Bd, q, R, P0) {
    var n = NX, i, j, k, m, it;
    var P = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    var Pn = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    for (i = 0; i < n; i++) P[i][i] = q[i];
    if (P0) { for (i = 0; i < n; i++) for (j = 0; j < n; j++) P[i][j] = P0[i][j]; }   // warm start

    var PB = new Float64Array(n), AtPB = new Float64Array(n);
    var PA = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
    for (it = 0; it < 3000; it++) {
      var denom = R;
      for (i = 0; i < n; i++) {
        var s = 0;
        for (j = 0; j < n; j++) s += P[i][j] * Bd[j];
        PB[i] = s;
        denom += Bd[i] * s;
      }
      if (!(denom > 1e-12)) return null;
      for (i = 0; i < n; i++) {
        var t = 0;
        for (k = 0; k < n; k++) t += Ad[k][i] * PB[k];
        AtPB[i] = t;
      }
      for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
          var v = 0;
          for (m = 0; m < n; m++) v += P[i][m] * Ad[m][j];
          PA[i][j] = v;
        }
      }
      var diff = 0;
      for (i = 0; i < n; i++) {
        for (j = 0; j <= i; j++) {
          var w = 0;
          for (k = 0; k < n; k++) w += Ad[k][i] * PA[k][j];
          w += -AtPB[i] * AtPB[j] / denom + (i === j ? q[i] : 0);
          var d = Math.abs(w - P[i][j]);
          if (d > diff) diff = d;
          Pn[i][j] = w; Pn[j][i] = w;
        }
      }
      for (i = 0; i < n; i++) for (j = 0; j < n; j++) P[i][j] = Pn[i][j];
      if (!isFinite(diff) || diff > 1e14) return null;
      if (diff < 1e-9) break;
    }
    for (i = 0; i < n; i++) {
      for (j = 0; j < n; j++) {
        if (!isFinite(P[i][j]) || Math.abs(P[i][j]) > 1e12) return null;
      }
    }
    return P;
  }

  /** Cholesky factorisation H = L L' (lower triangle); null if not pos. definite. */
  function cholesky(H, N) {
    var L = new Float64Array(N * N), i, j, k;
    for (i = 0; i < N; i++) {
      for (j = 0; j <= i; j++) {
        var s = H[i * N + j];
        for (k = 0; k < j; k++) s -= L[i * N + k] * L[j * N + k];
        if (i === j) {
          if (!(s > 1e-14)) return null;
          L[i * N + i] = Math.sqrt(s);
        } else {
          L[i * N + j] = s / L[j * N + j];
        }
      }
    }
    return L;
  }

  /** Solves L L' x = b (in place in x). */
  function cholSolve(L, b, x, N) {
    var i, k, s;
    for (i = 0; i < N; i++) {
      s = b[i];
      for (k = 0; k < i; k++) s -= L[i * N + k] * x[k];
      x[i] = s / L[i * N + i];
    }
    for (i = N - 1; i >= 0; i--) {
      s = x[i];
      for (k = i + 1; k < N; k++) s -= L[k * N + i] * x[k];
      x[i] = s / L[i * N + i];
    }
    return x;
  }

  function MpcController(cfg) {
    this.cfg = {
      N: 40,             // prediction horizon [steps]
      Ts: 0.02,          // control period [s]
      q: [10, 1, 100, 10],
      R: 0.5,
      umax: 15,          // input bound [N]
      terminal: true,    // include the terminal weight P
      Pdiag: null,       // null = P from the DARE, otherwise a manual diagonal
      mode: 'linear',    // 'linear' | 'nonlinear'
      maxSweeps: 120,
      tolRel: 1e-5
    };
    this.plant = Object.assign({}, IPM.model.DEFAULT_PLANT);
    this.dirty = true;
    this.Uprev = null;
    this.Pterm = null;
    if (cfg) this.configure(cfg);
  }

  MpcController.prototype.configure = function (cfg) {
    for (var k in cfg) {
      if (!Object.prototype.hasOwnProperty.call(cfg, k)) continue;
      if (k === 'plant') Object.assign(this.plant, cfg.plant);
      else if (k === 'q') this.cfg.q = cfg.q.slice();
      else if (k === 'Pdiag') this.cfg.Pdiag = cfg.Pdiag ? cfg.Pdiag.slice() : null;
      else this.cfg[k] = cfg[k];
    }
    this.dirty = true;
  };

  /**
   * Terminal weight: either the LQR solution for the model linearised about
   * the upright equilibrium (default), or a hand-supplied diagonal.
   */
  MpcController.prototype.rebuildTerminal = function () {
    if (!this.cfg.terminal) { this.Pterm = null; return; }
    if (this.cfg.Pdiag) {
      var Pm = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
      for (var d = 0; d < NX; d++) Pm[d][d] = this.cfg.Pdiag[d];
      this.Pterm = Pm;
      return;
    }
    var lin = IPM.model.linearizeUpright(this.plant);
    var dis = discretize(lin.A, lin.B, lin.c, this.cfg.Ts);
    var Ann = [], Bnn = new Float64Array(NX), i, j;
    for (i = 0; i < NX; i++) {
      Ann.push([]);
      for (j = 0; j < NX; j++) Ann[i].push(dis.Ad.d[i * NZ + j]);
      Bnn[i] = dis.Bd[i];
    }
    this.Pterm = dare(Ann, Bnn, this.cfg.q, this.cfg.R, this.Pterm);
  };

  MpcController.prototype.allocate = function () {
    var N = this.cfg.N;
    this.Phi = new Float64Array(N * NX * NZ);
    this.Gam = new Float64Array(N * NX * N);
    this.H = new Float64Array(N * N);
    this.g = new Float64Array(N);
    this.resid = new Float64Array(N);
    this.Xpred = new Float64Array(N * NX);
    this.Uunc = new Float64Array(N);
    this.Uout = new Float64Array(N);
    this.diagH = new Float64Array(N);
    this.gz = new Float64Array(NZ * N);
    this.gzTmp = new Float64Array(NZ * N);
    this.Tbuf = new Float64Array(NX * N);
    this.Uprev = new Float64Array(N);
    this.Nalloc = N;
  };

  /**
   * Build the prediction matrices Phi, Gamma and the Hessian H.
   * state is only needed in nonlinear mode (as the linearisation point).
   */
  MpcController.prototype.build = function (state) {
    var cfg = this.cfg, N = cfg.N, n = NX;
    if (this.Nalloc !== N) this.allocate();
    var Phi = this.Phi, Gam = this.Gam;
    var i, j, k, r, c;

    // --- 1) determine the linearisations -----------------------------------
    var steps = [];   // { Ad, Bd } per step
    if (cfg.mode === 'nonlinear') {
      // Reference trajectory: simulate the previous solution forward through
      // the nonlinear model and re-linearise at every point (RTI/SQP).
      var s = state.slice();
      for (k = 0; k < N; k++) {
        var uk = this.Uprev[k];
        var lin = IPM.model.linearizeAt(s, uk, this.plant);
        steps.push(discretize(lin.A, lin.B, lin.c, cfg.Ts));
        s = IPM.model.rk4(s, uk, 0, this.plant, cfg.Ts);
      }
      this.linInfo = { mode: 'nonlinear' };
    } else {
      var lin0 = IPM.model.linearizeUpright(this.plant);
      var d0 = discretize(lin0.A, lin0.B, lin0.c, cfg.Ts);
      for (k = 0; k < N; k++) steps.push(d0);
      this.linInfo = { mode: 'linear', A: lin0.A, B: lin0.B };
    }
    this.steps = steps;

    // --- 2) build Phi and Gamma recursively --------------------------------
    // Phi_k = C * Ad_k ... Ad_0 ;  Gamma block row k from
    //   Gz_k = Ad_k * Gz_{k-1},  column k := Bd_k
    var Pz = LA.eye(NZ);
    var gz = this.gz, gzT = this.gzTmp;
    gz.fill(0);
    for (k = 0; k < N; k++) {
      var Ad = steps[k].Ad, Bd = steps[k].Bd;
      Pz = LA.mul(Ad, Pz);
      for (r = 0; r < n; r++) {
        for (c = 0; c < NZ; c++) Phi[(k * n + r) * NZ + c] = Pz.d[r * NZ + c];
      }
      // gzT = Ad * gz  (only columns 0..k-1 are populated)
      for (r = 0; r < NZ; r++) {
        for (j = 0; j < k; j++) {
          var sacc = 0;
          for (c = 0; c < NZ; c++) sacc += Ad.d[r * NZ + c] * gz[c * N + j];
          gzT[r * N + j] = sacc;
        }
        gzT[r * N + k] = Bd[r];
      }
      for (r = 0; r < NZ; r++) for (j = 0; j <= k; j++) gz[r * N + j] = gzT[r * N + j];
      for (r = 0; r < n; r++) {
        for (j = 0; j <= k; j++) Gam[(k * n + r) * N + j] = gz[r * N + j];
      }
    }

    // --- 3) H = 2 (Gamma' Qbar Gamma + R I) --------------------------------
    var H = this.H, T = this.Tbuf, q = cfg.q, P = this.Pterm;
    H.fill(0);
    for (k = 0; k < N; k++) {
      var W = (k === N - 1 && P) ? P : null;
      for (r = 0; r < n; r++) {
        for (j = 0; j <= k; j++) {
          if (W) {
            var acc = 0;
            for (c = 0; c < n; c++) acc += W[r][c] * Gam[(k * n + c) * N + j];
            T[r * N + j] = acc;
          } else {
            T[r * N + j] = q[r] * Gam[(k * n + r) * N + j];
          }
        }
      }
      for (i = 0; i <= k; i++) {
        for (j = 0; j <= i; j++) {
          var a2 = 0;
          for (r = 0; r < n; r++) a2 += Gam[(k * n + r) * N + i] * T[r * N + j];
          H[i * N + j] += a2;
        }
      }
    }
    for (i = 0; i < N; i++) {
      for (j = 0; j < i; j++) {
        H[i * N + j] *= 2;
        H[j * N + i] = H[i * N + j];
      }
      H[i * N + i] = 2 * (H[i * N + i] + cfg.R);
      this.diagH[i] = H[i * N + i] > 1e-12 ? H[i * N + i] : 1e-12;
    }

    this.L = cholesky(H, N);
    this.dirty = false;
  };

  /** Run one control step. state: current state, xref: target state. */
  MpcController.prototype.step = function (state, xref) {
    var t0 = now();
    var cfg = this.cfg, N = cfg.N, n = NX;
    if (this.dirty) { this.rebuildTerminal(); }
    if (this.dirty || cfg.mode === 'nonlinear') this.build(state);

    var Phi = this.Phi, Gam = this.Gam, H = this.H, g = this.g, P = this.Pterm;
    var z0 = [state[0], state[1], state[2], state[3], 1];
    var i, j, k, r;

    // e_k = Phi_k z0 - xref  ->  g = 2 Gamma' Qbar e,  const = e' Qbar e
    var e = new Float64Array(N * n);
    var w = new Float64Array(N * n);
    var Jconst = 0;
    for (k = 0; k < N; k++) {
      var W = (k === N - 1 && P) ? P : null;
      for (r = 0; r < n; r++) {
        var s = 0;
        for (j = 0; j < NZ; j++) s += Phi[(k * n + r) * NZ + j] * z0[j];
        e[k * n + r] = s - xref[r];
      }
      for (r = 0; r < n; r++) {
        if (W) {
          var acc = 0;
          for (j = 0; j < n; j++) acc += W[r][j] * e[k * n + j];
          w[k * n + r] = acc;
        } else {
          w[k * n + r] = cfg.q[r] * e[k * n + r];
        }
        Jconst += w[k * n + r] * e[k * n + r];
      }
    }
    g.fill(0);
    for (k = 0; k < N; k++) {
      for (j = 0; j <= k; j++) {
        var gs = 0;
        for (r = 0; r < n; r++) gs += Gam[(k * n + r) * N + j] * w[k * n + r];
        g[j] += 2 * gs;
      }
    }

    // --- solve the QP ------------------------------------------------------
    var lo = -cfg.umax, hi = cfg.umax;
    var U = this.Uprev, Uunc = this.Uunc;
    var sweeps = 0, maxDelta = 0, unconstrained = false;

    if (this.L) {
      for (i = 0; i < N; i++) Uunc[i] = -g[i];
      cholSolve(this.L, Uunc, Uunc, N);
      unconstrained = true;
      for (i = 0; i < N; i++) {
        if (Uunc[i] < lo || Uunc[i] > hi) { unconstrained = false; break; }
      }
      for (i = 0; i < N; i++) U[i] = Uunc[i] < lo ? lo : (Uunc[i] > hi ? hi : Uunc[i]);
    } else {
      // fallback without Cholesky: shifted previous solution as the start
      for (i = 0; i < N - 1; i++) U[i] = U[i + 1];
      for (i = 0; i < N; i++) U[i] = U[i] < lo ? lo : (U[i] > hi ? hi : U[i]);
    }

    if (!unconstrained) {
      // residual res = H U + g, then projected coordinate descent
      var res = this.resid;
      for (i = 0; i < N; i++) {
        var rs = g[i];
        for (j = 0; j < N; j++) rs += H[i * N + j] * U[j];
        res[i] = rs;
      }
      var tol = cfg.tolRel * cfg.umax;
      for (var sw = 0; sw < cfg.maxSweeps; sw++) {
        maxDelta = 0;
        sweeps++;
        for (i = 0; i < N; i++) {
          var ui = U[i] - res[i] / this.diagH[i];
          if (ui < lo) ui = lo; else if (ui > hi) ui = hi;
          var d = ui - U[i];
          if (d !== 0) {
            U[i] = ui;
            for (j = 0; j < N; j++) res[j] += H[j * N + i] * d;
            var ad = d < 0 ? -d : d;
            if (ad > maxDelta) maxDelta = ad;
          }
        }
        if (maxDelta < tol) break;
      }
    }

    // cost J = 1/2 U'HU + g'U + const
    var J = Jconst;
    for (i = 0; i < N; i++) {
      var hu = 0;
      for (j = 0; j < N; j++) hu += H[i * N + j] * U[j];
      J += 0.5 * U[i] * hu + g[i] * U[i];
    }

    // predicted state sequence X = Phi z0 + Gamma U (for the visualisation)
    var X = this.Xpred;
    for (k = 0; k < N; k++) {
      for (r = 0; r < n; r++) {
        var xs = e[k * n + r] + xref[r];
        for (j = 0; j <= k; j++) xs += Gam[(k * n + r) * N + j] * U[j];
        X[k * n + r] = xs;
      }
    }

    // copy: U is about to be shifted for the warm start
    this.Uout.set(U);
    var result = {
      u: U[0],
      useq: this.Uout,
      xpred: X,
      cost: J,
      sweeps: sweeps,
      exact: unconstrained,
      converged: unconstrained || maxDelta < cfg.tolRel * cfg.umax,
      saturated: Math.abs(U[0]) >= cfg.umax - 1e-9,
      ms: now() - t0
    };

    // warm start for the next step: shift the solution by one sample
    this.shifted = true;
    var last = U[N - 1];
    for (i = 0; i < N - 1; i++) U[i] = U[i + 1];
    U[N - 1] = last;
    return result;
  };

  MpcController.prototype.reset = function () {
    if (this.Uprev) this.Uprev.fill(0);
    this.Pterm = null;
    this.dirty = true;
  };

  IPM.MpcController = MpcController;
  IPM.mpcInternals = { discretize: discretize, dare: dare, cholesky: cholesky };
})(typeof window !== 'undefined' ? window : globalThis);
